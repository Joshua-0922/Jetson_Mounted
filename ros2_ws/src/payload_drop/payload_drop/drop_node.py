"""Payload drop node (ground PC -> telemetry radio -> Pixhawk -> FMU PWM -> MG90D servo).

    ros2 run payload_drop drop --ros-args -p connection:=udpin:127.0.0.1:14560
    ros2 run payload_drop drop --ros-args -p connection:=udpin:127.0.0.1:14560 -p mode:=bench

mode:=manual  type 'drop' / 'close' / 'q' + Enter, or publish std_msgs/String 'drop' | 'close'
              on /payload_drop/command. Works armed (flight) or prearmed (bench).
mode:=bench   disarmed + COM_PREARM_MODE=2 only: N open/close cycles with random spacing, to build
              the latency distribution on the ground (props off).
Every command is logged with its PC send time and its estimate on the PX4 boot clock (TIMESYNC),
so `drop_latency <ulog> <session.json>` can split the delay into stages.
Published: /payload_drop/event (std_msgs/String, JSON per sent / ack / resent / no_ack).
"""
import json
import os
import random
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from . import core
from .link import Px4Link


class DropNode(Node):
    def __init__(self):
        super().__init__('payload_drop')
        P = self.declare_parameter
        self.url = P('connection', '/dev/ttyUSB0').value
        self.baud = int(P('baud', 57600).value)
        self.mode = P('mode', 'manual').value
        self.actuator_set = int(P('actuator_set', 1).value)
        self.open_value = float(P('open_value', 1.0).value)
        self.close_value = float(P('close_value', -1.0).value)
        self.auto_close_s = float(P('auto_close_s', 0.0).value)     # manual: 0 = stay open
        self.close_on_start = bool(P('close_on_start', True).value)
        self.bench_cycles = int(P('bench_cycles', 20).value)
        self.bench_open_s = float(P('bench_open_s', 1.5).value)
        self.bench_close_s = float(P('bench_close_s', 2.0).value)
        self.bench_jitter_s = float(P('bench_jitter_s', 0.5).value)
        self.log_dir = os.path.expanduser(P('log_dir', '~/drop_logs').value)
        skip_check = bool(P('skip_param_check', False).value)
        if self.mode not in ('manual', 'bench'):
            raise ValueError("mode must be 'manual' or 'bench'")
        if not 1 <= self.actuator_set <= 6:
            raise ValueError('actuator_set must be 1..6')

        log = self.get_logger()
        log.info(f'connecting to PX4 at {self.url} ({self.baud} baud)')
        self.link = Px4Link(self.url, self.baud, log=lambda s: log.info(s))
        if not self.link.wait_autopilot():
            raise RuntimeError('no PX4 heartbeat')
        self.link.start()
        for _ in range(6):                    # first clock-sync samples before any command
            self.link.timesync()
            time.sleep(0.15)

        params = self.link.get_params(core.first_pass_params())
        outs = core.find_outputs(params, core.FUNC_ACTUATOR_SET_BASE + self.actuator_set)
        if outs:
            params.update(self.link.get_params(core.channel_params(*outs[0])))
        self.px4_params = params
        self.cfg, errors, warnings = core.review_config(params, self.actuator_set, self.open_value,
                                                        self.close_value, self.mode)
        for w in warnings:
            log.warn(f'config: {w}')
        for e in errors:
            log.error(f'config: {e}')
        if errors and not skip_check:
            raise RuntimeError('PX4 output configuration not ready (see above); '
                               'skip_param_check:=true only for SITL')
        if self.cfg:
            log.info(f'servo on {self.cfg.bank}{self.cfg.channel}: open {self.cfg.open_us:.0f} us, '
                     f'close {self.cfg.close_us:.0f} us, disarmed {self.cfg.disarmed:.0f} us, '
                     f'{self.cfg.rate_hz:g} Hz, release sensor: {self.cfg.capture or "none"}')

        st = self.link.snapshot()
        if self.mode == 'bench' and st.armed:
            raise RuntimeError('bench mode must run DISARMED (props off, COM_PREARM_MODE=2)')

        self.stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        os.makedirs(self.log_dir, exist_ok=True)
        self.base = os.path.join(self.log_dir, f'drop_{self.mode}_{self.stamp}')
        self.jsonl = open(self.base + '.jsonl', 'w')
        self.pub = self.create_publisher(String, '/payload_drop/event', 10)
        self.ctl = core.DropController(self.link, self.actuator_set, self.open_value,
                                       self.close_value, on_event=self._on_event)
        self.queue = []                      # (monotonic time, kind) scheduled commands
        self.qlock = threading.Lock()
        self.done = False

        self.create_subscription(String, '/payload_drop/command', self._topic_cmd, 10)
        self.create_timer(0.02, self._tick)
        self.create_timer(0.5, self.link.timesync)
        self.create_timer(1.0, self.link.heartbeat)

        if self.close_on_start:
            self._schedule(0.0, 'close')
        if self.mode == 'bench':
            self._plan_bench()
        else:
            threading.Thread(target=self._keyboard, daemon=True).start()
        log.info(f'logging to {self.base}.jsonl')

    # ---------------------------------------------------------------- commands
    def _schedule(self, delay, kind):
        with self.qlock:
            self.queue.append((time.monotonic() + delay, kind))
            self.queue.sort()

    def _plan_bench(self):
        t = 1.0
        for _ in range(self.bench_cycles):
            # random spacing so the command phase is uniform w.r.t. the 50 Hz PWM frame and logger
            t += self.bench_close_s + random.uniform(0, self.bench_jitter_s)
            self._schedule(t, 'open')
            t += self.bench_open_s + random.uniform(0, self.bench_jitter_s)
            self._schedule(t, 'close')
        self._schedule(t + 1.5, 'quit')
        self.get_logger().info(f'bench: {self.bench_cycles} cycles, about {t:.0f} s')

    def _request(self, kind, source):
        if kind not in ('drop', 'open', 'close', 'quit'):
            self.get_logger().warn(f"unknown command '{kind}' from {source}")
            return
        kind = 'open' if kind == 'drop' else kind
        st = self.link.snapshot()
        if kind == 'open' and not st.armed and not self.px4_params.get('COM_PREARM_MODE'):
            self.get_logger().warn('disarmed and COM_PREARM_MODE=0: PX4 will not move the servo')
        self._schedule(0.0, kind)
        if kind == 'open' and self.auto_close_s > 0:
            self._schedule(self.auto_close_s, 'close')

    def _topic_cmd(self, msg):
        self._request(msg.data.strip().lower(), 'topic')

    def _keyboard(self):
        print("\n  commands: 'drop' (open the latch), 'close', 'q'\n", flush=True)
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd in ('q', 'quit', 'exit'):
                self._request('quit', 'keyboard')
                return
            if cmd:
                self._request(cmd, 'keyboard')

    def _tick(self):
        now = time.monotonic()
        while True:
            with self.qlock:
                if not self.queue or self.queue[0][0] > now:
                    break
                _, kind = self.queue.pop(0)
            if kind == 'quit':
                self.done = True
            else:
                if self.mode == 'bench' and self.link.snapshot().armed:
                    self.get_logger().error('vehicle armed during bench run: stopping')
                    with self.qlock:
                        self.queue.clear()
                    self.done = True
                    break
                self.ctl.send(kind)
        self.ctl.poll()
        if self.done and self.ctl.idle():
            raise SystemExit(0)

    def _on_event(self, ev, what):
        d = ev.as_dict()
        d['what'] = what
        self.jsonl.write(json.dumps(d) + '\n')
        self.jsonl.flush()
        self.pub.publish(String(data=json.dumps(d)))
        unc = '' if ev.sync_unc_us is None else f' (PX4 t={ev.px4_send_us / 1e6:.3f} s +/-{ev.sync_unc_us / 1e3:.1f} ms)'
        if what == 'sent':
            self.get_logger().info(f'#{ev.seq} {ev.kind} -> {ev.value:+.2f}{unc}')
        elif what == 'ack':
            self.get_logger().info(f'#{ev.seq} ACK result {ev.ack_result} after {ev.ack_rtt_ms:.1f} ms')
        else:
            self.get_logger().warn(f'#{ev.seq} {what}')

    # ---------------------------------------------------------------- output
    def finish(self):
        if getattr(self, 'jsonl', None) is None:
            return
        self.jsonl.close()
        self.jsonl = None
        sync = self.link.clock.best()
        meta = {
            'mode': self.mode, 'connection': self.url, 'actuator_set': self.actuator_set,
            'open_value': self.open_value, 'close_value': self.close_value,
            'output': None if self.cfg is None else self.cfg.__dict__,
            'px4_params': self.px4_params,
            'clock_sync': None if sync is None else {'offset_ns': sync[0], 'rtt_ns': sync[1],
                                                     'samples': self.link.clock.n_total},
            'events': [e.as_dict() for e in self.ctl.events],
            'statustext': [t for _, t in self.link.statustext],
        }
        with open(self.base + '.json', 'w') as fh:
            json.dump(meta, fh, indent=1)
        n_open = sum(e.kind == 'open' for e in self.ctl.events)
        rtts = sorted(e.ack_rtt_ms for e in self.ctl.events if e.ack_rtt_ms is not None)
        if rtts:
            self.get_logger().info(f'{len(self.ctl.events)} commands ({n_open} drops), ACK RTT '
                                   f'p50 {rtts[len(rtts) // 2]:.1f} ms, max {rtts[-1]:.1f} ms')
        self.get_logger().info(f'saved {self.base}.json -> drop_latency <ulog> {self.base}.json')


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = DropNode()
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        code = 130
    except (RuntimeError, ValueError) as e:
        print(f'payload_drop: {e}', file=sys.stderr)
        code = 1
    finally:
        if node is not None and hasattr(node, 'ctl'):
            node.finish()
        if node is not None and hasattr(node, 'link'):
            node.link.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

"""System-identification flight node (runs on the ground PC, talks MAVLink to PX4 over the radio).

    ros2 run system_identification sysid --ros-args -p connection:=/dev/ttyUSB0 -p test:=vertical

Flow: link + PX4 parameter check -> operator confirmation -> stream setpoints -> OFFBOARD + arm
-> takeoff -> maneuver segments at 10 Hz -> AUTO.LAND. Any abort condition switches to AUTO.LAND.
A pilot switching the RC mode away from OFFBOARD always wins: the node then stops commanding.

Published at 10 Hz (test frame: origin = takeoff point on the ground, x fwd / y left / z up):
  /sysid/setpoint  geometry_msgs/PointStamped   commanded point (x, y = NaN in velocity segments)
  /sysid/position  geometry_msgs/PointStamped   measured point
  /sysid/state     std_msgs/String              "<STATE> <segment>"
CSV (10 Hz) and a JSON summary go to log_dir; PX4 time_boot_ms in the CSV aligns with the ulog.
"""
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from std_msgs.msg import String

from . import maneuvers, safety
from .frames import TestFrame
from .px4_mavlink import Px4Link

RATE_HZ = 10.0


class SysIdNode(Node):
    def __init__(self):
        super().__init__('sysid')
        P = self.declare_parameter
        self.url = P('connection', '/dev/ttyUSB0').value
        self.baud = int(P('baud', 57600).value)
        self.test = P('test', 'hover').value
        self.hover_z = float(P('hover_z', 1.5).value)
        self.takeoff_mode = P('takeoff', 'auto').value          # 'auto' | 'pilot'
        self.land_cell_v = float(P('land_cell_v', 3.6).value)
        self.cells = int(P('battery_cells', 0).value)             # 0 -> read BAT1_N_CELLS
        self.log_dir = os.path.expanduser(P('log_dir', '~/sysid_logs').value)
        skip_check = bool(P('skip_param_check', False).value)
        confirm = bool(P('confirm', True).value)
        self.lim = safety.Limits(
            max_alt=float(P('max_alt', 2.8).value),
            box_xy=float(P('box_xy', 6.0).value),
            max_track_err=float(P('max_track_err', 2.5).value),
            max_tilt_deg=float(P('max_tilt_deg', 35.0).value),
            telem_timeout=float(P('telem_timeout', 1.0).value))
        if self.takeoff_mode not in ('auto', 'pilot'):
            raise ValueError("takeoff must be 'auto' or 'pilot'")

        self.segments = maneuvers.build(self.test, self.hover_z,
                                        pilot_takeoff=(self.takeoff_mode == 'pilot'))
        dur = maneuvers.total_duration(self.segments)

        self.pub_sp = self.create_publisher(PointStamped, '/sysid/setpoint', 10)
        self.pub_pos = self.create_publisher(PointStamped, '/sysid/position', 10)
        self.pub_state = self.create_publisher(String, '/sysid/state', 10)

        log = self.get_logger()
        log.info(f'connecting to PX4 at {self.url} ({self.baud} baud)')
        self.link = Px4Link(self.url, self.baud, log=lambda s: log.info(s))
        if not self.link.wait_autopilot():
            raise RuntimeError('no PX4 heartbeat')
        self.link.start()

        names = list(safety.REQUIRED_PARAMS) + list(safety.RECOMMENDED_PARAMS) + ['BAT1_N_CELLS']
        self.px4_params = self.link.get_params(names)
        errors, warnings = safety.review_params(self.px4_params)
        for w in warnings:
            log.warn(f'param: {w}')
        for e in errors:
            log.error(f'param: {e}')
        if errors and not skip_check:
            raise RuntimeError('PX4 failsafe parameters are not safe for this test (see above); '
                               'fix them in QGC, or skip_param_check:=true for SITL only')
        if self.cells <= 0:
            self.cells = int(self.px4_params.get('BAT1_N_CELLS') or 0)
        if self.cells <= 0:
            raise RuntimeError('battery cell count unknown: set battery_cells')

        st = self._wait_state(5.0)
        if st.armed:
            raise RuntimeError('vehicle is already armed: start the node on the ground, disarmed')

        log.info(f"test '{self.test}': {len(self.segments)} segments, {dur:.0f} s + landing, "
                 f'hover {self.hover_z:.1f} m, takeoff={self.takeoff_mode}, '
                 f'abort: alt>{self.lim.max_alt} m |xy|>{self.lim.box_xy} m '
                 f'tilt>{self.lim.max_tilt_deg} deg, land at {self.land_cell_v} V/cell '
                 f'({self.cells}S, now {st.voltage:.2f} V)')
        if self.takeoff_mode == 'auto' and confirm:
            print('\n  Props on, area clear, RC pilot ready to take over (switch out of OFFBOARD).')
            if input("  Type 'fly' to arm and take off: ").strip() != 'fly':
                raise RuntimeError('not confirmed')

        self.state = 'PRESTREAM' if self.takeoff_mode == 'auto' else 'WAIT_PILOT'
        self.reason = ''
        self.frame = None
        self.ground_d = st.ned[2]
        self.t_state = time.monotonic()
        self.t_last_cmd = 0.0
        self.seen_offboard = False
        self.seg_idx = 0
        self.seg_t0 = 0.0
        self.seg_p0 = None
        self.sp = None
        self.v_filt = st.voltage
        self.csv = None
        self.events = []
        self.stamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.create_timer(1.0, self.link.heartbeat)
        if self.takeoff_mode == 'pilot':
            log.info('pilot mode: arm and take off in Position mode, then switch to OFFBOARD')

    # ------------------------------------------------------------------------- helpers
    def _wait_state(self, timeout):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            st = self.link.snapshot()
            if st.t_pos and st.t_att and st.t_hb:
                return st
            time.sleep(0.1)
        raise RuntimeError('no LOCAL_POSITION_NED / ATTITUDE from PX4 (EKF not ready?)')

    def _set_state(self, new, reason=''):
        self.get_logger().info(f'{self.state} -> {new}' + (f': {reason}' if reason else ''))
        st = self.link.snapshot()
        self.events.append({'t_wall': time.time(), 'px4_boot_ms': st.boot_ms, 'from': self.state,
                            'to': new, 'reason': reason})
        self.state = new
        self.t_state = time.monotonic()

    def _abort(self, reason, normal=False):
        if self.state in ('LANDING', 'PILOT', 'DONE'):
            return
        self.reason = reason
        (self.get_logger().info if normal else self.get_logger().error)(f'LAND: {reason}')
        self.link.land()
        self.t_last_cmd = time.monotonic()
        self._set_state('LANDING', reason)

    def _define_frame(self, st):
        self.frame = TestFrame(st.ned[0], st.ned[1], self.ground_d, st.yaw)
        self._open_csv()

    def _measured(self, st):
        return self.frame.pos_from_ned(*st.ned)

    def _send(self, sp):
        f = self.frame
        pos = (None, None, f.pos_to_ned(0, 0, sp.z)[2]) if sp.xy_velocity_mode \
            else f.pos_to_ned(sp.x, sp.y, sp.z)
        self.link.send_setpoint_ned(pos, f.vel_to_ned(sp.vx, sp.vy, sp.vz), f.yaw_to_ned(sp.yaw),
                                    sp.xy_velocity_mode)

    # ------------------------------------------------------------------------- main loop
    def tick(self):
        now = time.monotonic()
        st = self.link.snapshot()
        age = now - min(st.t_pos, st.t_att)
        if not st.armed and self.state in ('WAIT_PILOT', 'PRESTREAM'):
            self.ground_d = st.ned[2]          # keep the ground reference until takeoff
        if st.offboard:
            self.seen_offboard = True
        if not math.isnan(st.voltage):     # ~3 s average: ignore sag during z steps
            if math.isnan(self.v_filt):
                self.v_filt = st.voltage
            else:
                self.v_filt += (st.voltage - self.v_filt) / (3.0 * RATE_HZ)

        getattr(self, f'_tick_{self.state.lower()}')(now, st, age)
        self._publish(st)

    def _tick_wait_pilot(self, now, st, age):
        # stream "stay where you are" so the RC switch to OFFBOARD is accepted and seamless
        self.link.send_setpoint_ned(st.ned, (0.0, 0.0, 0.0), st.yaw, False)
        if st.armed and st.offboard:
            self._define_frame(st)
            self._start_run(now)

    def _tick_prestream(self, now, st, age):
        if self.frame is None:
            self._define_frame(st)
        self._send(maneuvers.Setpoint(0.0, 0.0, 0.0))     # stay on the ground spot
        if now - self.t_state > 1.5:
            self._set_state('ENGAGE')

    def _tick_engage(self, now, st, age):
        self._send(maneuvers.Setpoint(0.0, 0.0, 0.0))
        if st.armed and st.offboard:
            self._start_run(now)
            return
        if now - self.t_state > 10.0:
            self.reason = 'could not enter OFFBOARD + arm within 10 s'
            self.get_logger().error(self.reason)
            self._set_state('DONE', self.reason)
            return
        if now - self.t_last_cmd > 1.0:
            self.t_last_cmd = now
            if not st.offboard:
                self.link.set_offboard()
            elif not st.armed:
                self.link.arm()

    def _start_run(self, now):
        self.seg_idx = -1
        self._set_state('RUN')
        self._next_segment(now, self.link.snapshot())

    def _next_segment(self, now, st):
        self.seg_idx += 1
        if self.seg_idx >= len(self.segments):
            self._abort('test complete', normal=True)
            return
        seg = self.segments[self.seg_idx]
        self.seg_t0 = now
        self.seg_p0 = self._measured(st) if seg.latch_measured else seg.start
        self.events.append({'t_wall': time.time(), 'px4_boot_ms': st.boot_ms,
                            'segment': seg.name, 'duration': seg.duration})
        self.get_logger().info(f'[{self.seg_idx + 1}/{len(self.segments)}] {seg.name} '
                               f'({seg.duration:.1f} s)')

    def _tick_run(self, now, st, age):
        if not st.armed:
            self._set_state('DONE', 'disarmed in flight')
            return
        if self.seen_offboard and not st.offboard:
            self.reason = 'pilot took over (mode left OFFBOARD)'
            self.get_logger().warn(self.reason + ': node stops commanding')
            self._set_state('PILOT', self.reason)
            return
        seg = self.segments[self.seg_idx]
        t = now - self.seg_t0
        if t >= seg.duration:
            self._next_segment(now, st)
            if self.state != 'RUN':
                return
            seg, t = self.segments[self.seg_idx], 0.0
        self.sp = seg.at(t, self.seg_p0)

        reason = safety.check(self.lim, self._measured(st), st.roll, st.pitch, age, self.sp)
        if reason:
            self._abort(reason)
            return
        if self.v_filt / self.cells < self.land_cell_v:
            self._abort(f'battery {self.v_filt:.2f} V ({self.v_filt / self.cells:.2f} V/cell) '
                        f'< {self.land_cell_v} V/cell', normal=(self.test == 'battery'))
            return
        self._send(self.sp)
        self._write_csv(st, seg.name)

    def _tick_landing(self, now, st, age):
        self._write_csv(st, 'landing')
        if not st.armed:
            self._set_state('DONE', 'landed and disarmed')
        elif st.auto_land:
            pass
        elif st.offboard or not self.seen_offboard:
            if now - self.t_last_cmd > 0.5:     # land command lost on the radio: repeat
                self.t_last_cmd = now
                self.link.land()
        else:
            self._set_state('PILOT', 'pilot changed mode during landing')

    def _tick_pilot(self, now, st, age):
        self._write_csv(st, 'pilot')
        if not st.armed:
            self._set_state('DONE', 'disarmed')

    def _tick_done(self, now, st, age):
        self._finish()
        raise SystemExit(0)

    # ------------------------------------------------------------------------- output
    def _publish(self, st):
        stamp = self.get_clock().now().to_msg()
        txt = self.state
        if self.state == 'RUN':
            txt += ' ' + self.segments[self.seg_idx].name
        self.pub_state.publish(String(data=txt))
        if self.frame is None:
            return
        m = PointStamped()
        m.header.stamp, m.header.frame_id = stamp, 'sysid_test'
        m.point.x, m.point.y, m.point.z = self._measured(st)
        self.pub_pos.publish(m)
        if self.state == 'RUN' and self.sp is not None:
            s = PointStamped()
            s.header = m.header
            nan = float('nan')
            s.point.x = nan if self.sp.x is None else self.sp.x
            s.point.y = nan if self.sp.y is None else self.sp.y
            s.point.z = self.sp.z
            self.pub_sp.publish(s)

    def _open_csv(self):
        os.makedirs(self.log_dir, exist_ok=True)
        self.base = os.path.join(self.log_dir, f'sysid_{self.test}_{self.stamp}')
        self.csv = open(self.base + '.csv', 'w')
        self.csv.write('t_wall,px4_boot_ms,state,segment,sp_x,sp_y,sp_z,sp_vx,sp_vy,sp_vz,sp_yaw,'
                       'x,y,z,vx,vy,vz,roll,pitch,yaw,voltage,current,armed,main_mode,sub_mode\n')
        self.get_logger().info(f'logging to {self.base}.csv')

    def _write_csv(self, st, seg_name):
        if not self.csv:
            return
        f = self.frame
        sp = self.sp if self.state == 'RUN' else None

        def g(v):
            return '' if v is None else f'{v:.4f}'
        spv = ['', '', '', '', '', '', ''] if sp is None else \
            [g(sp.x), g(sp.y), g(sp.z), g(sp.vx), g(sp.vy), g(sp.vz), g(sp.yaw)]
        x, y, z = f.pos_from_ned(*st.ned)
        vx, vy, vz = f.vel_from_ned(*st.vel_ned)
        row = [f'{time.time():.3f}', str(st.boot_ms), self.state, seg_name, *spv,
               *(f'{v:.4f}' for v in (x, y, z, vx, vy, vz, st.roll, st.pitch,
                                      f.yaw_from_ned(st.yaw), st.voltage, st.current)),
               str(int(st.armed)), str(st.main_mode), str(st.sub_mode)]
        self.csv.write(','.join(row) + '\n')

    def _finish(self):
        if not getattr(self, 'csv', None):
            return
        self.csv.close()
        f = self.frame
        meta = {
            'test': self.test, 'hover_z': self.hover_z, 'takeoff': self.takeoff_mode,
            'connection': self.url, 'result': self.reason,
            'frame': {'north0': f.n0, 'east0': f.e0, 'down_ground': f.d0, 'heading0_ned': f.psi},
            'limits': self.lim.__dict__, 'land_cell_v': self.land_cell_v, 'cells': self.cells,
            'px4_params': self.px4_params, 'events': self.events,
            'statustext': [t for _, t in self.link.statustext],
        }
        with open(self.base + '.json', 'w') as fh:
            json.dump(meta, fh, indent=1)
        self.get_logger().info(f'saved {self.base}.csv / .json ({self.reason})')
        self.csv = None

    def shutdown(self):
        """Ctrl+C / exception: if we are flying the vehicle, hand it to AUTO.LAND."""
        st = self.link.snapshot()
        if st.armed and getattr(self, 'state', None) in ('PRESTREAM', 'ENGAGE', 'RUN') and \
                (st.offboard or self.state != 'RUN'):
            self.get_logger().error('node stopping while in control: AUTO.LAND')
            self.reason = self.reason or 'node stopped'
            for _ in range(5):
                self.link.land()
                time.sleep(0.1)
        self._finish()
        self.link.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = SysIdNode()
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code or 0)
    except KeyboardInterrupt:
        code = 130
    except RuntimeError as e:
        print(f'sysid: {e}', file=sys.stderr)
        code = 1
    finally:
        if node is not None and hasattr(node, 'link'):
            node.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()

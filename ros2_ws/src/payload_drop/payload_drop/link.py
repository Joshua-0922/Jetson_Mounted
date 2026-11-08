"""MAVLink link for the drop node (ground PC -> telemetry radio, or UDP from a router / SITL).

Besides sending MAV_CMD_DO_SET_ACTUATOR it keeps two clocks related:
  - PC monotonic clock (time.monotonic_ns) for every send / ACK,
  - PX4 boot clock (hrt, same clock as the ulog timestamps) via MAVLink TIMESYNC.
So a PC send time can be placed on the ulog time axis.
"""
import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass

from pymavlink import mavutil

M = mavutil.mavlink
CMD_DO_SET_ACTUATOR = 187


class ClockSync:
    """PX4 boot clock <- PC monotonic clock, from TIMESYNC round trips.

    offset = px4_ns - midpoint(pc_send_ns, pc_recv_ns). The sample with the smallest round trip
    in the last `window_s` seconds is used: if the radio delay is asymmetric, its offset error is
    at most rtt/2 (this bound is reported with every estimate). The window stays short so crystal
    drift (~20-50 ppm) stays below ~1 ms.
    """

    def __init__(self, window_s=20.0):
        self.window_ns = int(window_s * 1e9)
        self.samples = deque()            # (pc_recv_ns, rtt_ns, offset_ns)
        self.n_total = 0

    def add(self, pc_send_ns, pc_recv_ns, px4_ns):
        rtt = pc_recv_ns - pc_send_ns
        if rtt <= 0 or px4_ns <= 0:
            return
        self.samples.append((pc_recv_ns, rtt, px4_ns - (pc_send_ns + pc_recv_ns) // 2))
        self.n_total += 1
        while self.samples and pc_recv_ns - self.samples[0][0] > self.window_ns:
            self.samples.popleft()

    def best(self):
        """(offset_ns, rtt_ns) of the min-RTT sample, or None."""
        if not self.samples:
            return None
        _, rtt, off = min(self.samples, key=lambda s: s[1])
        return off, rtt

    def to_px4_us(self, pc_ns):
        """(px4 boot time in us, +/- uncertainty in us) or (None, None)."""
        b = self.best()
        if b is None:
            return None, None
        off, rtt = b
        return (pc_ns + off) / 1000.0, rtt / 2000.0


@dataclass
class VehicleState:
    t_hb: float = 0.0
    armed: bool = False
    main_mode: int = 0
    sub_mode: int = 0
    landed_state: int = 0          # MAV_LANDED_STATE: 1 ground, 2 air, 3 takeoff, 4 landing
    voltage: float = float('nan')


class Px4Link:
    def __init__(self, url, baud=57600, target_system=1, log=print):
        self.log = log
        self.m = mavutil.mavlink_connection(url, baud=baud, source_system=245,
                                            source_component=M.MAV_COMP_ID_ONBOARD_COMPUTER,
                                            autoreconnect=True)
        self.ts = target_system
        self.tc = 1
        self.state = VehicleState()
        self.lock = threading.Lock()
        self.clock = ClockSync()
        self._ts_pending = {}             # our ts1 -> True (filters out replies to other GCSs)
        self._params = {}
        self._param_cv = threading.Condition()
        self.on_ack = None                # callback(pc_recv_ns, result) for DO_SET_ACTUATOR acks
        self.statustext = []
        self._stop = False
        self._thread = None

    # ---------------------------------------------------------------- receive
    def wait_autopilot(self, timeout=15.0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            hb = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if hb and hb.get_srcSystem() == self.ts and hb.get_srcComponent() == self.tc \
                    and hb.autopilot == M.MAV_AUTOPILOT_PX4:
                return True
        return False

    def start(self):
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()
        for msg_id, hz in ((M.MAVLINK_MSG_ID_SYS_STATUS, 1), (M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)):
            self.command(M.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, 1e6 / hz)

    def close(self):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        self.m.close()

    def _rx_loop(self):
        while not self._stop:
            try:
                msg = self.m.recv_match(blocking=True, timeout=0.2)
            except Exception as e:      # serial hiccup on the radio: keep going
                self.log(f'mavlink rx error: {e}')
                time.sleep(0.1)
                continue
            if msg is None:
                continue
            now_ns = time.monotonic_ns()   # take the receive time before anything else
            if msg.get_srcSystem() != self.ts or msg.get_srcComponent() != self.tc:
                continue
            self._handle(msg, now_ns)

    def _handle(self, msg, now_ns):
        t = msg.get_type()
        if t == 'TIMESYNC':
            # PX4 answers a request (tc1 == 0) with tc1 = hrt_absolute_time() * 1000, ts1 echoed
            if msg.tc1 != 0 and self._ts_pending.pop(msg.ts1, None):
                self.clock.add(msg.ts1, now_ns, msg.tc1)
        elif t == 'COMMAND_ACK':
            if msg.command == CMD_DO_SET_ACTUATOR and self.on_ack:
                self.on_ack(now_ns, msg.result)
            elif msg.command != CMD_DO_SET_ACTUATOR and msg.result != M.MAV_RESULT_ACCEPTED:
                self.log(f'PX4 rejected command {msg.command}: result {msg.result}')
        elif t == 'HEARTBEAT':
            with self.lock:
                s = self.state
                s.t_hb = time.monotonic()
                s.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                s.main_mode = (msg.custom_mode >> 16) & 0xFF
                s.sub_mode = (msg.custom_mode >> 24) & 0xFF
        elif t == 'EXTENDED_SYS_STATE':
            with self.lock:
                self.state.landed_state = msg.landed_state
        elif t == 'SYS_STATUS':
            with self.lock:
                self.state.voltage = msg.voltage_battery / 1000.0 \
                    if msg.voltage_battery != 65535 else float('nan')
        elif t == 'PARAM_VALUE':
            v = msg.param_value
            if msg.param_type in (M.MAV_PARAM_TYPE_INT32, M.MAV_PARAM_TYPE_UINT32):
                v = struct.unpack('<i', struct.pack('<f', v))[0]   # PX4 packs INT32 bytewise
            with self._param_cv:
                self._params[msg.param_id] = v
                self._param_cv.notify_all()
        elif t == 'STATUSTEXT':
            self.log(f'PX4: {msg.text}')
            self.statustext.append((time.time(), msg.text))

    def snapshot(self):
        with self.lock:
            return VehicleState(**self.state.__dict__)

    # ---------------------------------------------------------------- parameters
    def get_params(self, names, rounds=3, round_timeout=1.5):
        """Read parameters (rx thread running): request all, wait, re-request the missing ones.
        Names PX4 does not have (it does not answer them) map to None after `rounds`."""
        names = list(names)
        with self._param_cv:
            for n in names:
                self._params.pop(n, None)
        for _ in range(rounds):
            missing = [n for n in names if n not in self._params]
            if not missing:
                break
            for n in missing:
                self.m.mav.param_request_read_send(self.ts, self.tc, n.encode(), -1)
                time.sleep(0.005)          # do not burst a 57600 baud radio
            with self._param_cv:
                self._param_cv.wait_for(lambda: all(n in self._params for n in missing),
                                        timeout=round_timeout)
        with self._param_cv:
            return {n: self._params.get(n) for n in names}

    # ---------------------------------------------------------------- send
    def heartbeat(self):
        self.m.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def timesync(self):
        ts1 = time.monotonic_ns()
        if len(self._ts_pending) > 50:      # lost replies: forget the oldest
            for k in list(self._ts_pending)[:25]:
                self._ts_pending.pop(k, None)
        self._ts_pending[ts1] = True
        self.m.mav.timesync_send(0, ts1)

    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, confirmation=0):
        self.m.mav.command_long_send(self.ts, self.tc, cmd, confirmation, p1, p2, p3, p4, p5, p6, p7)

    def set_actuator(self, actuator_set, value, confirmation=0):
        """MAV_CMD_DO_SET_ACTUATOR: param1..6 = 'Peripheral via Actuator Set 1..6' (NaN = keep),
        param7 = 0 (index). Returns the PC monotonic send time (ns)."""
        p = [math.nan] * 6
        p[actuator_set - 1] = float(value)
        t_ns = time.monotonic_ns()
        self.command(CMD_DO_SET_ACTUATOR, *p, 0.0, confirmation=confirmation)
        return t_ns

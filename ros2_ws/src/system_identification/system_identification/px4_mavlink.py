"""Minimal PX4 MAVLink link for the ground PC (telemetry radio, or UDP from a router / SITL).

The Pixhawk USB port is dedicated to uXRCE-DDS (scripts/px4_usb_xrce.py), so during the
system-identification flights the ground PC talks MAVLink over the TELEM1 radio instead.
"""
import struct
import threading
import time
from dataclasses import dataclass

from pymavlink import mavutil

M = mavutil.mavlink

# PX4 custom modes (px4_custom_mode.h)
PX4_MAIN_POSCTL = 3
PX4_MAIN_AUTO = 4
PX4_MAIN_OFFBOARD = 6
PX4_SUB_AUTO_LAND = 6

# SET_POSITION_TARGET_LOCAL_NED type masks
_IGN_ACC = 64 | 128 | 256
_IGN_YAWRATE = 2048
MASK_POS_VEL_YAW = _IGN_ACC | _IGN_YAWRATE                 # x y z + velocity feed-forward + yaw
MASK_VELXY_POSZ_YAW = 1 | 2 | _IGN_ACC | _IGN_YAWRATE      # vx vy (z position held) + yaw


@dataclass
class VehicleState:
    t_pos: float = 0.0          # local monotonic receive time of the last LOCAL_POSITION_NED
    t_att: float = 0.0
    t_hb: float = 0.0
    boot_ms: int = 0            # PX4 time_boot_ms of the last LOCAL_POSITION_NED (aligns with the ulog)
    ned: tuple = (0.0, 0.0, 0.0)
    vel_ned: tuple = (0.0, 0.0, 0.0)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    armed: bool = False
    main_mode: int = 0
    sub_mode: int = 0
    landed_state: int = 0       # MAV_LANDED_STATE: 1 on ground, 2 in air, 3 takeoff, 4 landing
    voltage: float = float('nan')
    current: float = float('nan')

    @property
    def offboard(self):
        return self.main_mode == PX4_MAIN_OFFBOARD

    @property
    def auto_land(self):
        return self.main_mode == PX4_MAIN_AUTO and self.sub_mode == PX4_SUB_AUTO_LAND


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
        self._params = {}
        self._param_event = threading.Condition()
        self._stop = False
        self._t0 = time.monotonic()
        self._thread = None
        self.statustext = []

    # ---------------------------------------------------------------- connection / receive
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
        for msg_id, hz in ((M.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10),
                           (M.MAVLINK_MSG_ID_ATTITUDE, 10),
                           (M.MAVLINK_MSG_ID_SYS_STATUS, 2),
                           (M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 2)):
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
            except Exception as e:   # serial hiccup on the radio: keep going
                self.log(f'mavlink rx error: {e}')
                time.sleep(0.1)
                continue
            if msg is None or msg.get_srcSystem() != self.ts or msg.get_srcComponent() != self.tc:
                continue
            now = time.monotonic()
            t = msg.get_type()
            with self.lock:
                s = self.state
                if t == 'LOCAL_POSITION_NED':
                    s.t_pos, s.boot_ms = now, msg.time_boot_ms
                    s.ned = (msg.x, msg.y, msg.z)
                    s.vel_ned = (msg.vx, msg.vy, msg.vz)
                elif t == 'ATTITUDE':
                    s.t_att = now
                    s.roll, s.pitch, s.yaw = msg.roll, msg.pitch, msg.yaw
                elif t == 'HEARTBEAT':
                    s.t_hb = now
                    s.armed = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
                    s.main_mode = (msg.custom_mode >> 16) & 0xFF
                    s.sub_mode = (msg.custom_mode >> 24) & 0xFF
                elif t == 'EXTENDED_SYS_STATE':
                    s.landed_state = msg.landed_state
                elif t == 'SYS_STATUS':
                    s.voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else float('nan')
                    s.current = msg.current_battery / 100.0 if msg.current_battery >= 0 else float('nan')
            if t == 'PARAM_VALUE':
                v = msg.param_value
                if msg.param_type in (M.MAV_PARAM_TYPE_INT32, M.MAV_PARAM_TYPE_UINT32):
                    v = struct.unpack('<i', struct.pack('<f', v))[0]   # PX4 packs INT32 bytewise
                with self._param_event:
                    self._params[msg.param_id] = v
                    self._param_event.notify_all()
            elif t == 'STATUSTEXT':
                self.log(f'PX4: {msg.text}')
                self.statustext.append((now, msg.text))
            elif t == 'COMMAND_ACK' and msg.result != M.MAV_RESULT_ACCEPTED:
                self.log(f'PX4 rejected command {msg.command}: result {msg.result}')

    def snapshot(self):
        with self.lock:
            return VehicleState(**self.state.__dict__)

    # ---------------------------------------------------------------- parameters
    def get_params(self, names, timeout=10.0):
        """Read parameters (works with the rx thread running). Missing ones map to None."""
        out = {}
        pending = list(names)
        t_end = time.monotonic() + timeout
        while pending and time.monotonic() < t_end:
            name = pending[0]
            with self._param_event:
                self._params.pop(name, None)
            self.m.mav.param_request_read_send(self.ts, self.tc, name.encode(), -1)
            with self._param_event:
                self._param_event.wait_for(lambda: name in self._params, timeout=1.5)
                if name in self._params:
                    out[name] = self._params[name]
                    pending.pop(0)
                    continue
            pending.append(pending.pop(0))   # retry later, radio may have dropped it
        for name in names:
            out.setdefault(name, None)
        return out

    # ---------------------------------------------------------------- commands
    def heartbeat(self):
        self.m.mav.heartbeat_send(M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def command(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        self.m.mav.command_long_send(self.ts, self.tc, cmd, 0, p1, p2, p3, p4, p5, p6, p7)

    def set_offboard(self):
        self.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_OFFBOARD)

    def land(self):
        self.command(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                     PX4_MAIN_AUTO, PX4_SUB_AUTO_LAND)

    def arm(self):
        self.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1)

    def send_setpoint_ned(self, pos, vel, yaw, xy_velocity_mode):
        """pos/vel in NED. xy_velocity_mode: ignore x,y position (velocity control), keep z."""
        mask = MASK_VELXY_POSZ_YAW if xy_velocity_mode else MASK_POS_VEL_YAW
        n, e, d = (0.0 if v is None else v for v in pos)
        boot_ms = int((time.monotonic() - self._t0) * 1000) & 0xFFFFFFFF
        self.m.mav.set_position_target_local_ned_send(
            boot_ms, self.ts, self.tc, M.MAV_FRAME_LOCAL_NED, mask,
            n, e, d, vel[0], vel[1], vel[2], 0, 0, 0, yaw, 0)

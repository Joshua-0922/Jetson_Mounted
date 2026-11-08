"""Jetson <-> Pixhawk (uXRCE-DDS over serial) latency probe.

Three independent measurements, all on the Jetson system clock:

1. Command round trip (the "명령 -> 응답" path):
   publish VehicleCommand(ARM_DISARM, param1=0) while DISARMED and time the
   VehicleCommandAck. Commander treats disarm-while-disarmed as a no-op and
   answers ACCEPTED silently (no buzzer), so it is safe on the bench. The
   probe refuses to run while armed.
   With UXRCE_DDS_SYNCT=1 the client shifts outgoing timestamps into agent
   (Jetson) time, so ack.timestamp splits the RTT into uplink
   (Jetson -> commander) and downlink (commander -> Jetson).
2. Downlink one-way: vehicle_odometry receive time - timestamp / timestamp_sample.
3. PX4's own timesync RTT from /fmu/out/timesync_status.

Run it alone (no other node sending ARM_DISARM), props off.
"""
import csv
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (TimesyncStatus, VehicleCommand, VehicleCommandAck,
                          VehicleOdometry, VehicleStatus)

# PX4 timestamps above this are epoch-based (timesync applied); below, they are
# raw hrt time since boot and cannot be compared with the Jetson clock.
_EPOCH_US_MIN = 1_000_000_000_000_000


def _now_us():
    return time.time_ns() // 1000


def _stats(xs):
    if not xs:
        return 'n=0'
    a = sorted(x / 1000.0 for x in xs)  # us -> ms
    p95 = a[min(len(a) - 1, int(0.95 * len(a)))]
    return (f'n={len(a)} mean={statistics.fmean(a):.2f} p50={statistics.median(a):.2f} '
            f'p95={p95:.2f} max={a[-1]:.2f} ms')


class LatencyProbeNode(Node):
    def __init__(self):
        super().__init__('latency_probe_node')

        self.declare_parameter('probe_rate_hz', 5.0)
        self.declare_parameter('ack_timeout_s', 0.5)
        self.declare_parameter('report_period_s', 5.0)
        self.declare_parameter('csv_path', '')
        self._ack_timeout_us = int(self.get_parameter('ack_timeout_s').value * 1e6)
        self._csv_path = str(self.get_parameter('csv_path').value)

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # PX4 publishes VOLATILE; TRANSIENT_LOCAL here silently receives nothing.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack_cb, sub_qos)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, sub_qos)
        self.create_subscription(
            TimesyncStatus, '/fmu/out/timesync_status', self.timesync_cb, sub_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, sub_qos)

        self.armed = None          # unknown until the first vehicle_status
        self.pending_send_us = None
        self.lost = 0
        self.rtt, self.up, self.down = [], [], []
        self.odom_pub_age, self.odom_sample_age, self.odom_dt = [], [], []
        self.ts_rtt = []
        self._last_odom_recv = None
        self._unsynced_warned = False
        self._rows = []            # (kind, t_recv_us, value_us) for CSV

        self.create_timer(1.0 / self.get_parameter('probe_rate_hz').value, self.probe_cb)
        self.create_timer(self.get_parameter('report_period_s').value, self.report_cb)
        self.get_logger().info('Latency probe started (props off, vehicle disarmed).')

    # --- 1. command round trip -------------------------------------------
    def probe_cb(self):
        now = _now_us()
        if self.pending_send_us is not None:
            if now - self.pending_send_us < self._ack_timeout_us:
                return
            self.lost += 1
            self.pending_send_us = None
        if self.armed is not False:
            # None: no vehicle_status yet. True: a disarm probe would land the vehicle.
            return

        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0  # disarm -> no-op while disarmed, ACK ACCEPTED
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pending_send_us = _now_us()
        msg.timestamp = self.pending_send_us
        self.cmd_pub.publish(msg)

    def ack_cb(self, msg):
        t_recv = _now_us()
        if (msg.command != VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                or self.pending_send_us is None):
            return
        t_send, self.pending_send_us = self.pending_send_us, None
        rtt = t_recv - t_send
        self.rtt.append(rtt)
        self._rows.append(('cmd_rtt', t_recv, rtt))
        if msg.timestamp > _EPOCH_US_MIN:
            self.up.append(msg.timestamp - t_send)
            self.down.append(t_recv - msg.timestamp)
            self._rows.append(('cmd_up', t_recv, msg.timestamp - t_send))
            self._rows.append(('cmd_down', t_recv, t_recv - msg.timestamp))
        else:
            self._warn_unsynced()

    # --- 2. downlink one-way ---------------------------------------------
    def odom_cb(self, msg):
        t_recv = _now_us()
        if self._last_odom_recv is not None:
            self.odom_dt.append(t_recv - self._last_odom_recv)
        self._last_odom_recv = t_recv
        if msg.timestamp > _EPOCH_US_MIN:
            self.odom_pub_age.append(t_recv - msg.timestamp)
            self.odom_sample_age.append(t_recv - msg.timestamp_sample)
            self._rows.append(('odom_pub_age', t_recv, t_recv - msg.timestamp))
        else:
            self._warn_unsynced()

    # --- 3. PX4 timesync RTT ---------------------------------------------
    def timesync_cb(self, msg):
        self.ts_rtt.append(int(msg.round_trip_time))
        self._rows.append(('timesync_rtt', _now_us(), int(msg.round_trip_time)))

    def status_cb(self, msg):
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if armed and self.armed is not True:
            self.get_logger().warn('Vehicle ARMED: command probe paused.')
        self.armed = armed

    def _warn_unsynced(self):
        if not self._unsynced_warned:
            self._unsynced_warned = True
            self.get_logger().warn(
                'PX4 timestamps are not epoch-based: timesync is off or not converged '
                '(check UXRCE_DDS_SYNCT=1). Only RTT values are valid.')

    def report_cb(self):
        hz = (1e6 / statistics.fmean(self.odom_dt)) if self.odom_dt else 0.0
        log = self.get_logger().info
        log(f'[cmd RTT ] {_stats(self.rtt)} lost={self.lost}')
        log(f'[cmd up  ] {_stats(self.up)}')
        log(f'[cmd down] {_stats(self.down)}')
        log(f'[odom age] pub {_stats(self.odom_pub_age)} | sample {_stats(self.odom_sample_age)}'
            f' | rate {hz:.1f} Hz')
        log(f'[timesync] RTT {_stats(self.ts_rtt)}')

    def save_csv(self):
        if not self._csv_path:
            return
        with open(self._csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['kind', 't_recv_us', 'value_us'])
            w.writerows(self._rows)
        self.get_logger().info(f'Saved {len(self._rows)} samples to {self._csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = LatencyProbeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report_cb()
        node.save_csv()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

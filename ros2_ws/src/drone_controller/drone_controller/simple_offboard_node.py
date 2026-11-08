"""Minimal Jetson -> Pixhawk offboard control node (bench / first-flight check).

Sequence: stream setpoints -> OFFBOARD (+ARM if arm:=true) -> takeoff to
takeoff_alt over the start point -> hover -> forward velocity step -> hover -> land.

The velocity step also logs the response onset (10%) and 63% rise time from
vehicle_local_position, i.e. command -> vehicle response delay.

If the pilot leaves OFFBOARD (RC mode switch), the node stops re-requesting it
and goes idle. Props off for the first run.
"""

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

NAN = float('nan')


class SimpleOffboardNode(Node):
    def __init__(self):
        super().__init__('simple_offboard_node')

        self.declare_parameter('arm', False)          # False: pilot arms by RC
        self.declare_parameter('takeoff_alt', 2.0)    # m above start point
        self.declare_parameter('hover_s', 5.0)
        self.declare_parameter('step_vel', 0.5)       # m/s, +North
        self.declare_parameter('step_s', 3.0)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.auto_arm = bool(p('arm'))
        self.takeoff_alt = float(p('takeoff_alt'))
        self.hover_s = float(p('hover_s'))
        self.step_vel = float(p('step_vel'))
        self.step_s = float(p('step_s'))

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, sub_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.lpos_cb, sub_qos)

        self.status = None
        self.lpos = None
        self.state = 'WAIT_PX4'
        self.state_t0 = self.now_s()
        self.ticks = 0
        self.offboard_seen = False
        self.hold = None            # [x, y, z] NED position setpoint
        self.step_onset = None
        self.step_rise = None

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz (PX4 needs > 2 Hz)
        self.get_logger().info(f'Simple offboard node started (arm={self.auto_arm}).')

    # --- helpers ------------------------------------------------------------
    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_state(self, s):
        self.get_logger().info(f'{self.state} -> {s}')
        self.state, self.state_t0 = s, self.now_s()

    def in_state_for(self):
        return self.now_s() - self.state_t0

    def armed(self):
        return self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def in_offboard(self):
        return self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    # --- callbacks ----------------------------------------------------------
    def status_cb(self, msg):
        self.status = msg

    def lpos_cb(self, msg):
        self.lpos = msg
        if self.state != 'STEP':
            return
        t = self.in_state_for()
        if self.step_onset is None and msg.vx >= 0.1 * self.step_vel:
            self.step_onset = t
        if self.step_rise is None and msg.vx >= 0.63 * self.step_vel:
            self.step_rise = t

    # --- main loop ----------------------------------------------------------
    def loop(self):
        if self.status is None or self.lpos is None:
            return  # nothing from PX4 yet: agent/serial not up
        if self.state == 'WAIT_PX4':
            if not (self.lpos.xy_valid and self.lpos.z_valid):
                return
            self.hold = [self.lpos.x, self.lpos.y, self.lpos.z]
            self.set_state('STREAM')

        if self.state == 'DONE':
            return
        if self.state == 'LAND':  # AUTO_LAND owns the vehicle now; stop streaming
            if not self.armed():
                self.get_logger().info('Landed and disarmed.')
                self.set_state('DONE')
            return

        # Pilot took over: stop streaming so we never fight the RC switch.
        if self.offboard_seen and not self.in_offboard():
            self.get_logger().warn('Left OFFBOARD (pilot / failsafe) -> idle.')
            self.set_state('DONE')
            return

        self.ticks += 1
        velocity_mode = self.state == 'STEP'
        self.publish_mode(velocity_mode)
        if velocity_mode:
            self.publish_setpoint(vel=[self.step_vel, 0.0, 0.0])
        else:
            self.publish_setpoint(pos=self.hold)

        if self.in_offboard():
            self.offboard_seen = True

        if self.state == 'STREAM':
            # PX4 rejects OFFBOARD unless setpoints already flow; ~1 s of stream first.
            if self.ticks >= 20 and self.ticks % 10 == 0:
                if not self.in_offboard():
                    self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                elif not self.armed():
                    if self.auto_arm:
                        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    elif self.ticks % 100 == 0:
                        self.get_logger().info('In OFFBOARD, waiting for pilot to arm...')
            if self.in_offboard() and self.armed():
                self.hold = [self.lpos.x, self.lpos.y, self.lpos.z - self.takeoff_alt]
                self.set_state('TAKEOFF')

        elif self.state == 'TAKEOFF':
            if abs(self.lpos.z - self.hold[2]) < 0.3:
                self.set_state('HOVER')

        elif self.state == 'HOVER':
            if self.in_state_for() >= self.hover_s:
                self.step_onset = self.step_rise = None
                self.set_state('STEP')

        elif self.state == 'STEP':
            if self.in_state_for() >= self.step_s:
                fmt = lambda v: 'n/a' if v is None else f'{v * 1000:.0f} ms'  # noqa: E731
                self.get_logger().info(
                    f'Velocity step {self.step_vel} m/s: onset(10%)={fmt(self.step_onset)}, '
                    f'rise(63%)={fmt(self.step_rise)}')
                self.hold = [self.lpos.x, self.lpos.y, self.hold[2]]
                self.set_state('HOVER2')

        elif self.state == 'HOVER2':
            if self.in_state_for() >= self.hover_s:
                self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.set_state('LAND')

    # --- PX4 messages -------------------------------------------------------
    def publish_mode(self, velocity_mode):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = not velocity_mode
        msg.velocity = velocity_mode
        self.mode_pub.publish(msg)

    def publish_setpoint(self, pos=None, vel=None):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(v) for v in pos] if pos else [NAN, NAN, NAN]
        msg.velocity = [float(v) for v in vel] if vel else [NAN, NAN, NAN]
        # Velocity mode: yaw=NaN + yawspeed=0 holds heading. Position mode: keep start heading.
        msg.yaw = NAN if vel else float(self.lpos.heading)
        msg.yawspeed = 0.0 if vel else NAN
        self.sp_pub.publish(msg)

    def command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def land_if_flying(self):
        if self.status is not None and self.armed() and self.state not in ('LAND', 'DONE'):
            self.get_logger().warn('Shutdown while armed -> NAV_LAND')
            self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)


def main(args=None):
    # Keep the context alive after Ctrl-C so land_if_flying() can still publish.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SimpleOffboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.land_if_flying()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

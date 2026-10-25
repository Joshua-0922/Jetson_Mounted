import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS 2 메시지
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32
import math
# PX4 메시지
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleCommandAck

class DroneControllerNode(Node):
    def __init__(self):
        super().__init__('drone_controller_node')

        # --- [1] QoS 설정 ---
        # Publishers → PX4: BEST_EFFORT is sufficient; TRANSIENT_LOCAL is compatible
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Subscribers ← PX4: PX4 publishes VOLATILE; using TRANSIENT_LOCAL here
        # causes a silent DDS incompatibility and no messages are ever received.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --- [2] Publishers (To PX4) ---
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        # --- [3] Subscribers (From Brains) ---
        # A. 위치 제어 (순항 모드)
        self.pos_sub = self.create_subscription(
            Vector3, '/drone/cmd/position', self.position_command_callback, 10)

        # B. 속도 제어 (추적 모드)
        self.vel_sub = self.create_subscription(
            Twist, '/drone/cmd/velocity', self.velocity_command_callback, 10)

        # C. Heading for POSITION mode (ENU yaw, rad, CCW from East). Without it the position
        # setpoint carries yaw=NaN and the heading after takeoff is whatever PX4 holds -- the
        # Isaac drop policy observes its heading, so drop_policy pins it to the approach heading.
        self.yaw_sub = self.create_subscription(
            Float32, '/drone/cmd/yaw', self.yaw_command_callback, 10)
        self.target_yaw_ned = float('nan')

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, sub_qos)

        # #3: ack stream — lets us log *why* PX4 rejected an arm command
        # (the stuck-TAKEOFF / CRUISE-timeout cases) instead of silently
        # re-spamming the arm request with no diagnostic.
        self.command_ack_sub = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self.command_ack_callback, sub_qos)

        # --- [4] 내부 변수 ---
        self.control_mode = "POSITION"
        self._prev_control_mode = "POSITION"

        # 초기 고도 설정: 10m (NED 좌표계이므로 -10.0)
        self.target_pos = [0.0, 0.0, -10.0]
        self.target_vel = [0.0, 0.0, 0.0, 0.0]  # vx, vy, vz, yaw_speed

        # --- Velocity low-pass filter (wobble empirical check) ---
        # RL publishes step velocity setpoints at 10 Hz; this loop runs at 20 Hz.
        # An EMA on the published velocity smooths the step changes that make the
        # drone visibly wobble under RL control.
        #   alpha in (0, 1]:  v_filt = alpha*cmd + (1-alpha)*v_filt
        #   alpha >= 1.0  ->  pass-through (NO filtering) = raw baseline for A/B.
        # Time constant tau ~= dt * (1/alpha - 1); at 20 Hz, alpha=0.4 -> tau~=75 ms.
        # Default 1.0 = pass-through: identical to legacy behavior until opted in via
        # launch arg, so v14 (trained without the filter) is A/B'd fairly.
        self.declare_parameter('velocity_lpf_alpha', 1.0)
        self._vel_lpf_alpha = float(
            self.get_parameter('velocity_lpf_alpha').value)
        self._v_filt = [0.0, 0.0, 0.0, 0.0]

        self.vehicle_status = VehicleStatus()
        self.offboard_set_counter = 0
        self.px4_connected = False  # True after first vehicle_status received
        self._preflight_warned = False  # throttle "waiting on pre-flight" log to once
        # Instrumentation: measure EKF reconvergence latency after teleport reset.
        # _connect_time = first vehicle_status; _preflight_pass_logged throttles the
        # one-shot "pre_flight_checks_pass flipped True after Xs" report.
        self._connect_time = None
        self._preflight_pass_logged = False

        # --- [5] Main Loop (20Hz) ---
        self.timer = self.create_timer(0.05, self.cmdloop_callback)
        self.get_logger().info("Drone Controller Node Started! Hovering at 10m.")

    # ... (Callback 함수들은 기존과 동일, 생략 가능하지만 복붙 편의를 위해 유지) ...

    def position_command_callback(self, msg):
        self.control_mode = "POSITION"
        # ROS(ENU) -> PX4(NED): x->N, y->-E, z->-D
        self.target_pos = [msg.x, -msg.y, -msg.z] 

    def yaw_command_callback(self, msg):
        # ENU yaw (CCW from East) -> PX4 NED heading (CW from North)
        y = math.pi / 2.0 - float(msg.data)
        self.target_yaw_ned = math.atan2(math.sin(y), math.cos(y))

    def velocity_command_callback(self, msg):
        self.control_mode = "VELOCITY"
        # ROS(ENU) -> PX4(NED): vx->vx, vy->-vy, vz->-vz, yaw->-yaw
        self.target_vel = [msg.linear.x, -msg.linear.y, -msg.linear.z, -msg.angular.z]

    def vehicle_status_callback(self, msg):
        if not self.px4_connected:
            self.get_logger().info("PX4 connected — vehicle_status received.")
            self._connect_time = self.get_clock().now()
        self.px4_connected = True
        self.vehicle_status = msg

        # Instrumentation: report how long after PX4-connect the EKF reconverges
        # enough for PX4 to allow arming. This is the number that must beat the
        # bridge's arm_bail_timeout, and it tells us whether a late-flip ever
        # happens or whether these resets are permanently stuck (full-restart only).
        if (msg.pre_flight_checks_pass and not self._preflight_pass_logged
                and self._connect_time is not None):
            self._preflight_pass_logged = True
            dt = (self.get_clock().now() - self._connect_time).nanoseconds / 1e9
            self.get_logger().warn(
                f'PREFLIGHT-PASS: pre_flight_checks_pass flipped True '
                f'{dt:.1f}s after PX4-connect (arm-ready).')

    def command_ack_callback(self, msg):
        # #3: surface PX4's reason for rejecting an arm command. Without this
        # the controller just re-spams "Arming Drone" with no clue why the
        # drone never leaves TAKEOFF.
        if msg.command != VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            return
        if msg.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            return
        reason = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: 'TEMPORARILY_REJECTED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'DENIED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'UNSUPPORTED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'FAILED',
        }.get(msg.result, f'result={msg.result}')
        self.get_logger().warn(
            f'ARM REJECTED by PX4: {reason} '
            f'(pre_flight_checks_pass={self.vehicle_status.pre_flight_checks_pass}, '
            f'nav_state={self.vehicle_status.nav_state}, failsafe={self.vehicle_status.failsafe})')

    def cmdloop_callback(self):
        self.publish_offboard_control_mode()

        if self.control_mode == "POSITION":
            self.publish_position_setpoint(self.target_pos)
        elif self.control_mode == "VELOCITY":
            self.publish_velocity_setpoint(self._filter_velocity(self.target_vel))

        self._prev_control_mode = self.control_mode
        self.offboard_set_counter += 1

        # Wait until PX4 vehicle_status is actually received before arming.
        # Sending arm/offboard commands before PX4 is ready silently fails.
        if not self.px4_connected:
            return

        already_armed = (self.vehicle_status.arming_state == 2)   # ARMED
        in_offboard = (self.vehicle_status.nav_state == 14)        # OFFBOARD

        if already_armed and in_offboard:
            return  # Nothing to do

        # Warmup: wait 5 s after connection before first arm attempt.
        # EKF2 needs several seconds to initialise from GPS/IMU data. Arming
        # before EKF converges causes NaN velocity → 'Failsafe: blind land' →
        # NaN motor commands → Gazebo ODE AABB integer overflow → Gazebo crash.
        # At 20 Hz, 100 ticks = 5 s.  Arm retry every 2 s (40 ticks) thereafter.
        #
        # #2: the fixed 5 s warmup is necessary but NOT sufficient — after the
        # teleport reset the EKF often needs longer to reconverge, and arming
        # before pre_flight_checks_pass is set is rejected by PX4 (the dominant
        # cause of stuck-TAKEOFF / CRUISE timeouts). Gate the arm on PX4's own
        # arm-readiness flag instead of blindly spamming the request.
        warmup_ticks = 100
        if self.offboard_set_counter >= warmup_ticks and self.offboard_set_counter % 40 == 0:
            if not already_armed:
                if self.vehicle_status.pre_flight_checks_pass:
                    self.arm()
                elif not self._preflight_warned:
                    self._preflight_warned = True
                    self.get_logger().warn(
                        'Delaying arm — PX4 pre_flight_checks_pass=False '
                        '(EKF not yet reconverged after reset).')
        if self.offboard_set_counter % 10 == 0:
            if not in_offboard:
                self.engage_offboard_mode()

    def _filter_velocity(self, cmd):
        """EMA low-pass on the RL velocity setpoint to damp wobble.

        Snaps to the incoming command on the first VELOCITY tick (or when
        filtering is disabled) to avoid a ramp-from-zero lag when RL takes
        over from cruise; applies the exponential moving average thereafter.
        """
        a = self._vel_lpf_alpha
        if a >= 1.0 or self._prev_control_mode != "VELOCITY":
            self._v_filt = list(cmd)
        else:
            self._v_filt = [a * c + (1.0 - a) * f
                            for c, f in zip(cmd, self._v_filt)]
        return self._v_filt

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, pos_ned):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])]
        msg.velocity = [float('nan'), float('nan'), float('nan')]
        msg.yaw = self.target_yaw_ned
        self.trajectory_setpoint_pub.publish(msg)

    def publish_velocity_setpoint(self, vel_ned):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
        # yaw must be NaN for PX4 to follow yawspeed. Left at the message default (0.0) PX4 pins
        # the heading to North and ignores yawspeed -- every RL yaw-rate command was silently
        # dropped before 2026-10-11 (found in the drop_policy sim-to-sim check).
        msg.yaw = float('nan')
        msg.yawspeed = float(vel_ned[3])
        self.trajectory_setpoint_pub.publish(msg)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("--> Switching to OFFBOARD Mode")

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("--> Arming Drone")

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
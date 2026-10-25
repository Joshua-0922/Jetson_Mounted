#!/usr/bin/env python3
"""drop_mission_node -- flies one drop episode with the Isaac Lab policy, on PX4 (SITL or real).

Reuses the existing ws architecture:
  * drone_controller_node  arms, holds OFFBOARD, converts /drone/cmd/{position,velocity} to PX4
                           TrajectorySetpoints and applies the 20 Hz EMA (velocity_lpf_alpha = 0.4,
                           exactly the Isaac ActionCfg LPF). Its command convention is (North, West, Up).
  * this node              replaces mission_manager + rl_navigation for the trained policy:
        CLIMB    position hold at (0, 0, cruise_alt) over the spawn point (= PX4 local origin)
        CRUISE   velocity along the approach heading at cruise_speed (the Isaac handoff state)
        POLICY   handoff at marker_dist from the target: policy at 10 Hz, release check at the
                 odometry rate, drop command after release_delay (the mean the predictor assumes)
        HOLD     after landing / timeout: hover in place; result JSON written, node exits
Frames: PX4 NED/FRD -> ENU/FLU via policy_core.px4_to_enu. Target and spawn are given in the Gazebo
(ENU) world; local = world - spawn. Altitude above ground = -z_ned + alt_offset (body origin height
at rest, 0.24 m for x500).
"""
import json
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float32

from drop_policy import policy_core as pc


class DropMissionNode(Node):
    def __init__(self):
        super().__init__("drop_mission_node")
        P = lambda n, v: self.declare_parameter(n, v).value  # noqa: E731
        self.models_path = P("models_path", "")
        self.target_w = np.array([P("target_x", 20.0), P("target_y", 0.0)])
        self.spawn_w = np.array([P("spawn_x", 0.0), P("spawn_y", 0.0)])
        self.heading = math.radians(P("heading_deg", 0.0))
        self.cruise_speed = P("cruise_speed", 4.0)
        self.cruise_alt = P("cruise_alt", 10.0)
        self.marker_dist = P("marker_dist", 20.0)
        self.alt_offset = P("alt_offset", 0.24)
        self.use_residual = P("use_residual", False)
        self.ema_alpha = P("ema_alpha", 0.3)
        self.release_delay = P("release_delay", pc.RELEASE_DELAY_MEAN)
        self.result_path = P("result_path", "/tmp/drop_result.json")
        self.tag = P("tag", "")
        self.episode_timeout = P("episode_timeout", 30.0)
        # "gz_truth": Gazebo ground-truth odometry (RTK-grade, sim only) | "px4": PX4 EKF estimate
        self.state_source = P("state_source", "gz_truth")

        m = dict(np.load(self.models_path))
        self.target_local = self.target_w - self.spawn_w
        self.policy = pc.DropPolicy(m, self.target_local, use_residual=self.use_residual, ema_alpha=self.ema_alpha)

        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_cb, sub_qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.status_cb, sub_qos)
        self.create_subscription(Odometry, "/drop/payload/odom", self.payload_cb, 10)
        self.create_subscription(Odometry, "/drop/drone/odom", self.truth_cb, 10)
        self.truth = None
        self.pos_pub = self.create_publisher(Vector3, "/drone/cmd/position", 10)
        self.vel_pub = self.create_publisher(Twist, "/drone/cmd/velocity", 10)
        self.drop_pub = self.create_publisher(Empty, "/drop/payload/drop_cmd", 10)
        self.yaw_pub = self.create_publisher(Float32, "/drone/cmd/yaw", 10)

        self.state = "WAIT"
        self.state_t = self.now()
        self.armed_offboard = False
        self.est = None              # (pos, vel, R, ang) ENU local, altitude above ground in pos[2]
        self.payload = None          # world ENU position of the payload
        self.mount_log = []          # drone-payload offsets while carried (mount check)
        self.handoff_t = None
        self.drop_cmd_t = None
        self.release_state = None
        self.trace = []
        self.result = None
        self.create_timer(0.1, self.tick)       # 10 Hz = policy rate
        self.get_logger().info(f"drop_mission {self.tag}: target_local={self.target_local}, residual={self.use_residual}")

    # ----------------------------------------------------------------- callbacks
    def status_cb(self, msg):
        self.armed_offboard = (msg.arming_state == 2 and msg.nav_state == 14)

    def now(self):
        """Seconds on the node clock: wall time on hardware, Gazebo /clock with use_sim_time (SITL may run
        slower than real time, e.g. while rendering video; PX4 is lockstepped to the same clock)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg):
        if not (np.all(np.isfinite(msg.position)) and np.all(np.isfinite(msg.velocity))):
            return
        pos, vel, R, ang = pc.px4_to_enu(msg.position, msg.velocity, msg.q, msg.angular_velocity)
        pos[2] += self.alt_offset
        self.px4_est = (pos, vel, R, ang)
        if self.state_source == "px4":
            self.on_state(pos, vel, R, ang)

    def truth_cb(self, msg):
        p, q, tw = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        R = pc.quat_to_rot((q.w, q.x, q.y, q.z))                       # FLU body -> ENU world
        pos = np.array([p.x, p.y, p.z]) - np.array([*self.spawn_w, 0.0])  # local; z = body origin above ground
        vel = R @ np.array([tw.linear.x, tw.linear.y, tw.linear.z])       # odometry twist is in the body frame
        ang = np.array([tw.angular.x, tw.angular.y, tw.angular.z])
        self.truth = (pos, vel, R, ang)
        self.sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.state_source == "gz_truth":
            self.on_state(pos, vel, R, ang)

    def on_state(self, pos, vel, R, ang):
        self.est = (pos, vel, R, ang)
        if self.state == "POLICY" and not self.policy.released:
            if self.policy.release_check(pos, vel, R, ang):
                self.release_state = dict(t=self.now() - self.handoff_t, pos=pos.tolist(), vel=vel.tolist())
                self.drop_cmd_t = self.now() + self.release_delay
                self.get_logger().info(f"RELEASE decided at {pos.round(2)} v={vel.round(2)} -> drop in {self.release_delay:.2f}s")
        if self.drop_cmd_t is not None and self.now() >= self.drop_cmd_t:
            self.drop_pub.publish(Empty())
            self.policy.attached = False
            self.release_state["detach_pos"] = pos.tolist()
            self.release_state["detach_vel"] = vel.tolist()
            self.drop_cmd_t = None

    def payload_cb(self, msg):
        p = msg.pose.pose.position
        self.payload = np.array([p.x, p.y, p.z])
        if self.policy.attached and self.truth is not None and self.state in ("CRUISE", "POLICY"):
            drone_w = np.array([*(self.truth[0][:2] + self.spawn_w), self.truth[0][2]])
            self.mount_log.append((self.payload - drone_w).tolist())
        if self.state == "POLICY" and not self.policy.attached and self.result is None \
                and p.z <= pc.PAYLOAD_GROUND_Z + pc.PAYLOAD_LAND_EPS:
            self.land_sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.finish("landed", impact_w=self.payload[:2])

    def payload_rel(self):
        """payload - body origin (world ENU), valid at hover where topic timing does not matter."""
        if self.payload is None or self.truth is None:
            return None
        drone_w = np.array([*(self.truth[0][:2] + self.spawn_w), self.truth[0][2]])
        return self.payload - drone_w

    # ----------------------------------------------------------------- commands (drone_controller convention)
    def cmd_position(self, local_enu_xyz):
        """Position setpoint given in OUR local frame (ENU, origin = spawn, z = altitude above ground).
        PX4's local frame has its own origin (in SITL: the Gazebo world origin; on the real vehicle:
        wherever EKF2 initialised), so convert through the measured PX4 - state offset."""
        self.yaw_pub.publish(Float32(data=float(self.heading)))   # Isaac: the drone faces the approach heading
        d = np.asarray(local_enu_xyz, float).copy()
        px4 = getattr(self, "px4_est", None)
        if px4 is not None and self.est is not None:
            d = d + (px4[0] - self.est[0])
        d[2] -= self.alt_offset                                   # PX4 local z = altitude - body height at rest
        self.pos_pub.publish(Vector3(x=float(d[1]), y=float(-d[0]), z=float(d[2])))

    def cmd_velocity(self, v_enu, yaw_rate_enu=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = float(v_enu[1]), float(-v_enu[0]), float(v_enu[2])
        t.angular.z = float(yaw_rate_enu)
        self.vel_pub.publish(t)

    # ----------------------------------------------------------------- state machine
    def goto(self, s):
        self.get_logger().info(f"{self.state} -> {s}")
        self.state, self.state_t = s, self.now()

    def tick(self):
        if self.est is None:
            return
        pos, vel, R, ang = self.est
        u = np.array([math.cos(self.heading), math.sin(self.heading)])
        yaw = pc.euler_xyz(R)[2]
        yaw_err = math.atan2(math.sin(self.heading - yaw), math.cos(self.heading - yaw))
        if self.state == "WAIT":
            self.cmd_position([0.0, 0.0, self.cruise_alt])
            if self.armed_offboard:
                self.goto("CLIMB")
        elif self.state == "CLIMB":
            self.cmd_position([0.0, 0.0, self.cruise_alt])
            if abs(pos[2] - self.cruise_alt) < 0.3 and np.linalg.norm(vel) < 0.3 and abs(yaw_err) < math.radians(5) \
                    and self.now() - self.state_t > 3.0:
                carried = self.payload_rel()
                if carried is not None:
                    self.hover_mount = carried.tolist()
                self.goto("CRUISE")
        elif self.state == "CRUISE":
            vz = float(np.clip(1.0 * (self.cruise_alt - pos[2]), -1.0, 1.0))
            self.cmd_velocity([*(u * self.cruise_speed), vz], float(np.clip(2.0 * yaw_err, -1.0, 1.0)))
            along = float((self.target_local - pos[:2]) @ u)
            if along <= self.marker_dist:
                self.policy.reset(np.array([*(u * self.cruise_speed), 0.0]))
                self.handoff_t = self.now()
                self.handoff_sim_t = getattr(self, "sim_t", None)
                self.handoff_state = dict(pos=pos.tolist(), vel=vel.tolist(), along=along)
                self.goto("POLICY")
        elif self.state == "POLICY":
            v, yr, obs = self.policy.step(pos, vel, R, ang)
            self.cmd_velocity(v, yr)
            t = self.now() - self.handoff_t
            _, d_imp, _ = self.policy.ccip(pos, vel)
            px4 = getattr(self, "px4_est", None)
            self.trace.append(dict(px4_pos=None if px4 is None else px4[0].round(3).tolist(), t=round(t, 2), pos=pos.round(3).tolist(), vel=vel.round(3).tolist(),
                                   cmd=np.round(v, 3).tolist(), d_impact=round(d_imp, 3),
                                   residual=np.round(self.policy.residual_m, 3).tolist(), wants_drop=self.policy.wants_drop,
                                   obs=np.round(obs, 4).tolist()))
            if t > self.episode_timeout and self.result is None:
                self.finish("timeout_released" if self.policy.released else "timeout_no_release")
        elif self.state == "HOLD":
            self.cmd_position([*self.hold_xy, self.hold_alt])

    def finish(self, outcome, impact_w=None):
        pos = self.est[0]
        self.hold_xy, self.hold_alt = pos[:2].copy(), pos[2]
        res = dict(tag=self.tag, outcome=outcome, use_residual=self.use_residual,
                   target_w=self.target_w.tolist(), spawn_w=self.spawn_w.tolist(),
                   heading_deg=math.degrees(self.heading), cruise_speed=self.cruise_speed, cruise_alt=self.cruise_alt,
                   marker_dist=self.marker_dist, handoff=getattr(self, "handoff_state", None),
                   release=self.release_state, deliver_time_s=self.now() - self.handoff_t if self.handoff_t else None,
                   mount_offset_mean=np.mean(self.mount_log, axis=0).tolist() if self.mount_log else None,
                   mount_offset_std=np.std(self.mount_log, axis=0).tolist() if self.mount_log else None,
                   mount_samples=len(self.mount_log), hover_mount_offset=getattr(self, "hover_mount", None),
                   handoff_sim_t=getattr(self, "handoff_sim_t", None), land_sim_t=getattr(self, "land_sim_t", None),
                   trace=self.trace)
        if impact_w is not None:
            res["impact_w"] = np.asarray(impact_w).tolist()
            res["impact_err"] = float(np.linalg.norm(np.asarray(impact_w) - self.target_w))
        self.result = res
        os.makedirs(os.path.dirname(self.result_path) or ".", exist_ok=True)
        with open(self.result_path, "w") as f:
            json.dump(res, f, indent=1)
        self.get_logger().info(f"RESULT {self.tag}: {outcome} err={res.get('impact_err')} -> {self.result_path}")
        self.goto("HOLD")
        self.create_timer(3.0, lambda: (_ for _ in ()).throw(SystemExit))


def main(args=None):
    rclpy.init(args=args)
    node = DropMissionNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

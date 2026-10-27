#!/usr/bin/env python3
"""Measure Pixhawk <-> Jetson link latency over uXRCE-DDS (for sim2real delay modelling).

Start the agent first:  MicroXRCEAgent serial --dev /dev/pixhawk -b 921600   (USB; /dev/ttyACM0 without the udev rule)
Then:                   python3 scripts/measure_link_latency.py --duration 30 --json out.json

Three independent measurements:
  1. RTT (clock-free): sends a no-op VehicleCommand (unknown id -> commander replies UNSUPPORTED)
     and times the VehicleCommandAck. Includes serial both ways + agent + DDS + commander scheduling,
     so it is an upper bound on 2x the transport delay.
  2. Uplink (PX4 -> Jetson): arrival time minus VehicleOdometry.timestamp / timestamp_sample.
     With UXRCE_DDS_SYNCT=1 (default) PX4 shifts outgoing stamps into the agent clock (Jetson
     system time), so the difference is the one-way delay +- the timesync error (~RTT asymmetry).
  3. Odometry rate and inter-arrival jitter (clock-free).
Never arms or changes modes; safe with props on, but do it with props off anyway.
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import TimesyncStatus, VehicleCommand, VehicleCommandAck, VehicleOdometry

NOOP_CMD = 31337   # not a MAV_CMD; commander's default branch acks it UNSUPPORTED, nothing else reacts


def now_us():
    return time.time_ns() // 1000


def stats(xs):
    if not xs:
        return None
    a = np.asarray(xs, dtype=float)
    return {"n": int(a.size), "mean": float(a.mean()), "std": float(a.std()), "min": float(a.min()),
            "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)), "max": float(a.max())}


class LinkLatency(Node):
    def __init__(self, ping_hz):
        super().__init__("link_latency")
        sub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.odom_cb, sub_qos)
        self.create_subscription(VehicleCommandAck, "/fmu/out/vehicle_command_ack", self.ack_cb, sub_qos)
        self.create_subscription(TimesyncStatus, "/fmu/out/timesync_status", self.ts_cb, sub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", pub_qos)
        self.odom_arrival, self.uplink, self.sample_age = [], [], []
        self.rtt, self.px4_rtt, self.offset = [], [], []
        self.pending = {}          # seq -> send time (us); seq is carried in param1
        self.seq = 0
        self.sent = 0
        if ping_hz > 0:
            self.create_timer(1.0 / ping_hz, self.ping)

    def odom_cb(self, msg):
        t = now_us()
        self.odom_arrival.append(t)
        if msg.timestamp:
            self.uplink.append(t - msg.timestamp)
        if msg.timestamp_sample:
            self.sample_age.append(t - msg.timestamp_sample)

    def ts_cb(self, msg):
        if msg.source_protocol == TimesyncStatus.SOURCE_PROTOCOL_DDS:
            self.px4_rtt.append(msg.round_trip_time)
            self.offset.append(msg.estimated_offset)

    def ping(self):
        self.seq += 1
        msg = VehicleCommand()
        msg.command = NOOP_CMD
        msg.param1 = float(self.seq)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        t = now_us()
        msg.timestamp = t
        self.pending[self.seq] = t
        self.cmd_pub.publish(msg)
        self.sent += 1

    def ack_cb(self, msg):
        t = now_us()
        if msg.command != NOOP_CMD or not self.pending:
            return
        # VehicleCommandAck has no param echo; acks come back in order, so match the oldest pending
        seq = min(self.pending)
        self.rtt.append(t - self.pending.pop(seq))
        for s in [s for s, t0 in self.pending.items() if t - t0 > 1_000_000]:   # lost
            del self.pending[s]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=30.0, help="seconds to measure")
    ap.add_argument("--ping-hz", type=float, default=20.0, help="no-op command rate for RTT (0 = off)")
    ap.add_argument("--json", help="write raw stats to this file")
    a = ap.parse_args()

    rclpy.init()
    n = LinkLatency(a.ping_hz)
    t_end = time.time() + a.duration
    print(f"measuring for {a.duration:.0f} s ...")
    while time.time() < t_end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)

    arr = np.diff(n.odom_arrival) / 1000.0 if len(n.odom_arrival) > 1 else []
    up = [u / 1000.0 for u in n.uplink]
    out = {
        "odom_rate_hz": (len(n.odom_arrival) - 1) / ((n.odom_arrival[-1] - n.odom_arrival[0]) / 1e6)
        if len(n.odom_arrival) > 1 else 0.0,
        "odom_interarrival_ms": stats(list(arr)),
        "uplink_ms(arrival - timestamp)": stats(up),
        "sample_age_ms(arrival - timestamp_sample)": stats([u / 1000.0 for u in n.sample_age]),
        "cmd_ack_rtt_ms": stats([r / 1000.0 for r in n.rtt]),
        "cmd_sent": n.sent, "cmd_acked": len(n.rtt),
        "px4_timesync_rtt_ms": stats([r / 1000.0 for r in n.px4_rtt]),
        "px4_timesync_offset_us_last": n.offset[-1] if n.offset else None,
    }
    if not n.odom_arrival:
        print("!! no /fmu/out/vehicle_odometry received - is MicroXRCEAgent connected?")
    elif up and abs(np.median(up)) > 1000:
        print("!! uplink delay > 1 s: timestamps not synced (UXRCE_DDS_SYNCT=0?) - ignore the uplink rows")
    for k, v in out.items():
        if isinstance(v, dict):
            print(f"{k:44s} mean {v['mean']:7.2f}  p50 {v['p50']:7.2f}  p95 {v['p95']:7.2f}  "
                  f"p99 {v['p99']:7.2f}  max {v['max']:7.2f}  (n={v['n']})")
        else:
            print(f"{k:44s} {v}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=2)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

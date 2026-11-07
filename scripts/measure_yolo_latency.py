#!/usr/bin/env python3
"""Measure IMX219 -> YOLO detection latency (for sim2real perception-delay modelling).

Start the pipeline first:  ros2 launch yolo_detector yolo.launch.py
Then:                      python3 scripts/measure_yolo_latency.py --duration 60 --json out.json

Time origin is the sensor start-of-frame (SOF): imx219_camera stamps header.stamp with the Argus
sensor timestamp (first row arriving from the sensor), recovered per frame from nvarguscamerasrc's
show-latency log. All stages are measured against that one stamp on the same host clock, so no
cross-machine sync is involved.
  1. Consumer-side latency: arrival of /yolo/detections here minus header.stamp
     (what a controller node on the Jetson would see).
  2. Per-stage breakdown from /yolo/timing (published by the detector for every processed frame):
     SOF->detector receive, imgmsg->cv2, predict (Ultralytics pre+infer+post), SOF->detections publish.
  3. Output rate, inter-arrival jitter and dropped camera frames (stamp gaps > 1.5 frame periods).
  4. Age of the newest detection when sampled at a random time (latency + sample-and-hold), which
     is what a fixed-rate controller actually consumes.
To refer latency to the exposure instead of SOF, add exposure/2 (row 0) and, for a target at
image row y, (y / H) * readout; see the model block in the JSON.
"""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from vision_msgs.msg import Detection2DArray

FRAME_PERIOD_MS = 1000.0 / 30.0


def stats(xs):
    if not xs:
        return None
    a = np.asarray(xs, dtype=float)
    a = a[np.isfinite(a)]
    if a.size == 0:
        return None
    return {"n": int(a.size), "mean": float(a.mean()), "std": float(a.std()), "min": float(a.min()),
            "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)), "max": float(a.max())}


class YoloLatency(Node):
    def __init__(self):
        super().__init__("yolo_latency")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Detection2DArray, "/yolo/detections", self.det_cb, qos)
        self.create_subscription(Float64MultiArray, "/yolo/timing", self.timing_cb, qos)
        self.arrival_ns, self.stamp_ns, self.latency_ms, self.n_boxes = [], [], [], []
        self.timing = []

    def det_cb(self, msg):
        now = self.get_clock().now().nanoseconds
        st = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.arrival_ns.append(now)
        self.stamp_ns.append(st)
        self.latency_ms.append((now - st) / 1e6)
        self.n_boxes.append(len(msg.detections))

    def timing_cb(self, msg):
        self.timing.append(list(msg.data))


def held_age(arrival_ns, stamp_ns, n=20000, seed=0):
    """Age of the newest available detection at uniformly random query times."""
    arr = np.asarray(arrival_ns, dtype=np.int64)
    st = np.asarray(stamp_ns, dtype=np.int64)
    q = np.random.default_rng(seed).uniform(arr[1], arr[-1], n).astype(np.int64)
    idx = np.searchsorted(arr, q, side="right") - 1
    return list((q - st[idx]) / 1e6)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=60.0, help="seconds to measure")
    ap.add_argument("--warmup", type=float, default=3.0, help="seconds to discard first")
    ap.add_argument("--label", default="", help="free-text condition label stored in the JSON")
    ap.add_argument("--json", help="write stats to this file")
    a = ap.parse_args()

    rclpy.init()
    node = YoloLatency()
    t_end = time.monotonic() + a.warmup
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.arrival_ns.clear(); node.stamp_ns.clear(); node.latency_ms.clear()
    node.n_boxes.clear(); node.timing.clear()
    t_end = time.monotonic() + a.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)

    if len(node.arrival_ns) < 10:
        print("too few detections messages; is `ros2 launch yolo_detector yolo.launch.py` running?")
        return 1

    tm = np.asarray(node.timing, dtype=float) if node.timing else np.empty((0, 5))
    stamps = np.asarray(node.stamp_ns, dtype=np.int64)
    stamp_gap = np.diff(stamps) / 1e6
    inter = np.diff(np.asarray(node.arrival_ns, dtype=np.int64)) / 1e6
    skipped = int(np.sum(np.round(stamp_gap / FRAME_PERIOD_MS) - 1).clip(0)) if stamp_gap.size else 0

    out = {
        "label": a.label,
        "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "duration_s": a.duration,
        "time_origin": "sensor start-of-frame (Argus sensor timestamp, first row)",
        "consumer_sof_to_detection_ms": stats(node.latency_ms),
        "stages_ms": {
            "sof_to_detector_receive": stats(list(tm[:, 0])),
            "imgmsg_to_cv2": stats(list(tm[:, 1])),
            "predict": stats(list(tm[:, 2])),
            "sof_to_detections_publish": stats(list(tm[:, 3])),
            "sof_to_annotated_publish": stats(list(tm[:, 4])),
        },
        "detection_rate_hz": len(inter) / (sum(inter) / 1000.0) if len(inter) else None,
        "inter_arrival_ms": stats(list(inter)),
        "camera_frames_processed": int(len(stamps)),
        "camera_frames_skipped": skipped,
        "skip_ratio": skipped / (skipped + len(stamps)),
        "held_age_ms (latency + sample-and-hold, random query time)": stats(
            held_age(node.arrival_ns, node.stamp_ns)),
        "boxes_per_frame_mean": float(np.mean(node.n_boxes)),
    }

    s = out["consumer_sof_to_detection_ms"]
    print(f"detections: {out['camera_frames_processed']} msgs, {out['detection_rate_hz']:.1f} Hz, "
          f"camera frames skipped {skipped} ({100 * out['skip_ratio']:.1f}%)")
    print(f"SOF -> detection at consumer: p50 {s['p50']:.1f}  p95 {s['p95']:.1f}  p99 {s['p99']:.1f}  "
          f"max {s['max']:.1f}  mean {s['mean']:.1f} +- {s['std']:.1f} ms")
    for k, v in out["stages_ms"].items():
        if v:
            print(f"  {k:28s} p50 {v['p50']:6.1f}  p95 {v['p95']:6.1f}  max {v['max']:6.1f} ms")
    h = out["held_age_ms (latency + sample-and-hold, random query time)"]
    print(f"held age (random query):      p50 {h['p50']:.1f}  p95 {h['p95']:.1f}  max {h['max']:.1f} ms")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=2)
        print(f"wrote {a.json}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

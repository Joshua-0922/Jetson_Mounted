#!/usr/bin/env python3
"""YOLO inference load on the Jetson (camera stand-in: a looped video file), with its own latency stats.

    ~/yolo-venv/bin/python scripts/yolo_load.py --model ~/yolo_models/yolo11n.engine --fps 30 --duration 70
    ~/yolo-venv/bin/python scripts/yolo_load.py --fps 0      # as fast as possible (worst-case load)

Run it next to measure_link_latency.py to see how perception load affects the PX4 link.
Per frame it records Ultralytics' preprocess / inference / postprocess times and the end-to-end time
from frame grab to result (what a perception node would add before acting on a detection).
"""
import argparse
import json
import time

import cv2
import numpy as np
from ultralytics import YOLO

DEFAULT_VIDEO = "/usr/src/jetson_multimedia_api/data/Video/sample_outdoor_car_1080p_10fps.h264"


def stats(xs):
    a = np.asarray(xs, dtype=float)
    return {"n": int(a.size), "mean": float(a.mean()), "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)), "p99": float(np.percentile(a, 99)), "max": float(a.max())}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="yolo11n.pt", help=".pt (PyTorch) or .engine (TensorRT)")
    ap.add_argument("--video", default=DEFAULT_VIDEO)
    ap.add_argument("--fps", type=float, default=30.0, help="frame rate to feed (0 = unthrottled)")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--duration", type=float, default=70.0)
    ap.add_argument("--json", help="write stats here")
    a = ap.parse_args()

    model = YOLO(a.model, task="detect")
    cap = cv2.VideoCapture(a.video)
    ok, frame = cap.read()
    if not ok:
        raise SystemExit(f"cannot read {a.video}")
    for _ in range(5):                      # warm-up (CUDA context, TensorRT bindings)
        model.predict(frame, imgsz=a.imgsz, device=0, half=True, verbose=False)

    period = 1.0 / a.fps if a.fps > 0 else 0.0
    pre, inf, post, e2e, ndet = [], [], [], [], []
    t_start = time.monotonic()
    next_t = t_start
    print(f"running {a.model} on {a.video} at {'max' if not period else a.fps} fps for {a.duration:.0f} s", flush=True)
    while time.monotonic() - t_start < a.duration:
        if period:
            now = time.monotonic()
            if now < next_t:
                time.sleep(next_t - now)
            next_t += period
        t_grab = time.monotonic()
        ok, frame = cap.read()
        if not ok:                          # loop the clip
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = cap.read()
        r = model.predict(frame, imgsz=a.imgsz, device=0, half=True, verbose=False)[0]
        e2e.append((time.monotonic() - t_grab) * 1000)
        pre.append(r.speed["preprocess"])
        inf.append(r.speed["inference"])
        post.append(r.speed["postprocess"])
        ndet.append(len(r.boxes))
    el = time.monotonic() - t_start
    out = {"model": a.model, "target_fps": a.fps, "achieved_fps": len(e2e) / el, "frames": len(e2e),
           "preprocess_ms": stats(pre), "inference_ms": stats(inf), "postprocess_ms": stats(post),
           "grab_to_result_ms": stats(e2e), "detections_per_frame_mean": float(np.mean(ndet))}
    print(f"achieved {out['achieved_fps']:.1f} fps, {len(e2e)} frames, {out['detections_per_frame_mean']:.1f} det/frame")
    for k in ("preprocess_ms", "inference_ms", "postprocess_ms", "grab_to_result_ms"):
        v = out[k]
        print(f"  {k:18s} p50 {v['p50']:6.2f}  p95 {v['p95']:6.2f}  p99 {v['p99']:6.2f}  max {v['max']:6.2f}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()

"""nvarguscamerasrc capture with per-frame sensor start-of-frame (SOF) time.

nvarguscamerasrc stamps buffers with the pipeline clock when it *pushes* them
(do-timestamp), ~40-50 ms after the sensor started sending the frame. With
show-latency=true it also logs, per capture,
    "capture id N latency X ms"      (X = SOF -> GstBuffer push, from Argus metadata)
at DEBUG level. We hook the GStreamer log, pair each log line with the buffer
pushed at the same instant (they coincide to ~0.1 ms) and recover
    SOF[CLOCK_MONOTONIC] = PTS + base_time - X.
The pipeline clock is GstSystemClock = CLOCK_MONOTONIC, the same domain as the
Argus sensor timestamp.
"""
import collections
import re
import threading
import time

import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst  # noqa: E402

_LAT_RE = re.compile(r'capture id (\d+) latency ([\d.]+) ms')
_MATCH_TOL_NS = 3_000_000


def build_pipeline(sensor_id, width, height, fps, flip_method, wbmode, saturation):
    return (
        f'nvarguscamerasrc sensor-id={sensor_id} wbmode={wbmode} '
        f'saturation={saturation} show-latency=true ! '
        f'video/x-raw(memory:NVMM),width={width},height={height},'
        f'framerate={fps}/1,format=NV12 ! '
        f'nvvidconv flip-method={flip_method} ! video/x-raw,format=BGRx ! '
        'videoconvert ! video/x-raw,format=BGR ! '
        'appsink name=sink drop=true max-buffers=1 sync=false'
    )


class GstCapture:
    def __init__(self, pipeline_str):
        Gst.init(None)
        self._log = collections.deque(maxlen=64)  # (push_mono_ns, capture_id, latency_ms)
        self._lock = threading.Lock()
        Gst.debug_set_active(True)
        Gst.debug_remove_log_function(None)          # silence default stderr logger
        Gst.debug_set_threshold_for_name('nvarguscamerasrc', Gst.DebugLevel.DEBUG)
        Gst.debug_add_log_function(self._on_log, None)

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name('sink')
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('pipeline failed to start')
        self.unmatched = 0

    def _on_log(self, cat, level, file, func, line, obj, msg, *user):
        if cat.get_name() != 'nvarguscamerasrc':
            return
        m = _LAT_RE.search(msg.get())
        if m:
            with self._lock:
                self._log.append((time.clock_gettime_ns(time.CLOCK_MONOTONIC),
                                  int(m.group(1)), float(m.group(2))))

    def read(self, timeout_s=1.0):
        """Return (frame_bgr, sof_mono_ns or None, push_mono_ns) or None on timeout."""
        sample = self.sink.emit('try-pull-sample', int(timeout_s * Gst.SECOND))
        if sample is None:
            return None
        buf = sample.get_buffer()
        s = sample.get_caps().get_structure(0)
        w, h = s.get_value('width'), s.get_value('height')
        ok, info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            stride = len(info.data) // h
            frame = np.ndarray((h, stride), np.uint8, info.data)[:, :w * 3] \
                .reshape(h, w, 3).copy()
        finally:
            buf.unmap(info)

        push = buf.pts + self.pipeline.get_base_time()
        with self._lock:
            entries = list(self._log)
        sof = None
        if entries:
            t, _, lat_ms = min(entries, key=lambda e: abs(e[0] - push))
            if abs(t - push) <= _MATCH_TOL_NS:
                sof = push - int(lat_ms * 1e6)
        if sof is None:
            self.unmatched += 1
        return frame, sof, push

    def release(self):
        self.pipeline.set_state(Gst.State.NULL)
        try:
            Gst.debug_remove_log_function(self._on_log)
        except Exception:  # noqa: BLE001
            pass

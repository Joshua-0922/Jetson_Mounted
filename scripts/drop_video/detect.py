"""Find servo-open events and the payload release in the ROI stacks (frames are 1/240 s apart).

For every servo motion burst:
  baseline  = mean ROI image over frames [b-70, b-20] (vehicle at rest before the command)
  d_roi(f)  = mean |roi[f] - baseline|  (change from rest, robust to single-frame noise)
  f_servo   = first frame where d_horn exceeds  thr = max(mu + K*sd, floor) for HOLD frames
  f_slider  = same on the latch slider ROI
  f_release = same on the payload body ROI (only for opening events)
Opening vs closing: an opening ends far from the "closed" horn template, a closing ends near it.
"""
import json
import sys

import numpy as np

FPS = 240.0
K, HOLD = 6.0, 3


def load(prefix):
    meta = json.load(open(prefix + '_meta.json'))
    n = meta['frames']
    rois = {k: np.load(f'{prefix}_{k}.npy', mmap_mode='r')[:n] for k in ('horn', 'slider', 'payload')}
    return rois, n


def frame_motion(stack, step=1):
    """mean |x[t] - x[t-step]| per frame (float32), computed in chunks."""
    n = stack.shape[0]
    out = np.zeros(n, np.float32)
    for s in range(step, n, 4000):
        e = min(n, s + 4000)
        a = stack[s:e].astype(np.int16)
        b = stack[s - step:e - step].astype(np.int16)
        out[s:e] = np.abs(a - b).mean(axis=(1, 2))
    return out


def bursts(sig, thr, min_gap):
    idx = np.flatnonzero(sig > thr)
    if idx.size == 0:
        return []
    groups, start, prev = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - prev > min_gap:
            groups.append((start, prev))
            start = i
        prev = i
    groups.append((start, prev))
    return groups


def onset(stack, b, search_end, base=(70, 20), floor=1.0):
    """First frame f in [b-30, search_end) where change-from-baseline exceeds the threshold for HOLD frames."""
    ref = stack[b - base[0]:b - base[1]].astype(np.float32)
    mean_img = ref.mean(axis=0)
    d_base = np.abs(ref - mean_img).mean(axis=(1, 2))
    thr = max(d_base.mean() + K * d_base.std(), d_base.mean() + floor)
    seg = stack[b - 30:search_end].astype(np.float32)
    d = np.abs(seg - mean_img).mean(axis=(1, 2))
    run = 0
    for i, v in enumerate(d):
        run = run + 1 if v > thr else 0
        if run >= HOLD:
            return b - 30 + i - HOLD + 1, d, thr
    return None, d, thr


def main(prefix):
    rois, n = load(prefix)
    horn = rois['horn']
    m = frame_motion(horn, step=2)
    # robust noise level of the horn motion signal
    med = np.median(m)
    mad = np.median(np.abs(m - med)) + 1e-6
    thr = med + 12 * 1.4826 * mad
    raw = bursts(m, thr, min_gap=240)            # bursts closer than 1 s belong together
    raw = [(s, e) for s, e in raw if e - s >= 3]
    # closed template: horn image well before the first burst
    closed_tpl = horn[max(0, raw[0][0] - 200):raw[0][0] - 40].astype(np.float32).mean(axis=0)
    events = []
    for s, e in raw:
        if s < 120 or e + 120 >= n:
            continue
        after = horn[e + 40:e + 100].astype(np.float32).mean(axis=0)
        before = horn[s - 70:s - 20].astype(np.float32).mean(axis=0)
        d_after = float(np.abs(after - closed_tpl).mean())
        d_before = float(np.abs(before - closed_tpl).mean())
        kind = 'open' if d_after > d_before else 'close'
        ev = {'burst': [int(s), int(e)], 'kind': kind, 'd_after_closed': d_after, 'd_before_closed': d_before}
        if kind == 'open':
            f_servo, _, th_h = onset(horn, s, s + 120)
            f_slider, _, _ = onset(rois['slider'], s, s + 240)
            f_rel, dp, th_p = onset(rois['payload'], s, s + 480)
            # confirm a real drop: payload ROI still far from its rest image 0.5 s later
            ref = rois['payload'][s - 70:s - 20].astype(np.float32).mean(axis=0)
            late = float(np.abs(rois['payload'][min(n - 1, s + 150)].astype(np.float32) - ref).mean())
            ev.update({'f_servo': f_servo, 'f_slider': f_slider, 'f_release': f_rel,
                       'payload_change_late': late, 'thr_payload': float(th_p)})
        events.append(ev)
    json.dump({'n': n, 'horn_thr': float(thr), 'events': events}, open(prefix + '_events.json', 'w'),
              indent=1, default=int)
    for ev in events:
        if ev['kind'] == 'open':
            fs, fr = ev['f_servo'], ev['f_release']
            dt = None if fs is None or fr is None else (fr - fs) / FPS * 1e3
            print(f"OPEN  burst {ev['burst']}  servo {fs}  slider {ev['f_slider']}  release {fr}  "
                  f"-> {dt if dt is None else round(dt, 1)} ms  late_change {ev['payload_change_late']:.1f}")
        else:
            print(f"close burst {ev['burst']}")


if __name__ == '__main__':
    main(sys.argv[1])

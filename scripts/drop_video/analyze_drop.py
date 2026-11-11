"""Drop latency from the 240 fps videos: servo motion onset -> payload release, per trial.

  servo onset  t_s : horn ROI change-from-rest; line through the first points above threshold,
                     extrapolated back to the rest level (sub-frame).
  release      t_r : payload vertical shift s(t) tracked by 1-D edge correlation; fit
                     s = 1/2 a (t - t_r)^2 (free fall from rest) on the early fall (sub-frame).
  delay D = t_r - t_s  [ms]  (1 frame = 1/240 s = 4.167 ms)
"""
import json
import math
import sys

import numpy as np
from scipy import optimize, stats

FPS = 240.0
VIDEOS = {'v1': ('c1', 'video 20261108_160339', range(3, 13)),
          'v2': ('c2', 'video 20261108_162057', range(17, 27))}


def periodic_opens(events, period=3600, tol=90):
    """Keep openings that form the 15 s chain and actually dropped something."""
    opens = [e for e in events if e['kind'] == 'open' and e['f_servo'] is not None
             and e['payload_change_late'] > 20]
    fs = [e['f_servo'] for e in opens]
    keep = []
    for e in opens:
        f = e['f_servo']
        if any(abs(abs(f - g) - k * period) < tol for g in fs if g != f for k in (1, 2, 3)):
            keep.append(e)
    return keep


def change_curve(stack, f0, f1, base):
    ref = stack[base[0]:base[1]].astype(np.float32)
    mu = ref.mean(axis=0)
    d_base = np.abs(ref - mu).mean(axis=(1, 2))
    seg = stack[f0:f1].astype(np.float32)
    return np.abs(seg - mu).mean(axis=(1, 2)), d_base.mean(), d_base.std()


def servo_onset(horn, f_servo):
    base = (f_servo - 70, f_servo - 15)
    f0 = f_servo - 15
    d, mu, sd = change_curve(horn, f0, f_servo + 20, base)
    thr = mu + max(4 * sd, 0.3)
    above = np.flatnonzero(d > thr)
    # first run of 3 consecutive frames above threshold
    for i in above:
        if i + 2 < len(d) and (d[i:i + 3] > thr).all():
            break
    else:
        return None, None
    pts = np.arange(i, min(i + 4, len(d)))
    slope, icpt = np.polyfit(pts, d[pts], 1)
    t0 = (mu - icpt) / slope if slope > 0 else i
    t0 = min(max(t0, i - 2.0), float(i))      # never earlier than 2 frames before the crossing
    return f0 + t0, f0 + i


def edge_profile(img, cols):
    p = img[:, cols[0]:cols[1]].astype(np.float32).mean(axis=1)
    p = np.convolve(p, np.ones(3) / 3, mode='same')
    return np.gradient(p)


def track_shift(pcol, f_from, n, ref_frames, cols, max_shift=70):
    """Vertical shift (px, down +) of the payload relative to the rest image, sub-pixel.
    Template = payload body + its bottom edge only (the static blocks above are excluded)."""
    ref = pcol[ref_frames[0]:ref_frames[1]].astype(np.float32).mean(axis=0)
    g0 = edge_profile(ref, cols)
    L = len(g0)
    r_bot = 60 + int(np.argmin(g0[60:L - max_shift - 5]))     # bright body -> darker background
    r0, r1 = max(28, r_bot - 100), r_bot + 6
    tpl = g0[r0:r1]
    out = []
    for f in range(f_from, f_from + n):
        g = edge_profile(pcol[f], cols)
        prev = out[-1][0] if out else 0.0
        lo_s, hi_s = max(0, int(prev) - 1), min(max_shift, int(prev) + 9)
        scores = np.full(max_shift + 1, -1.0)
        for s in range(lo_s, hi_s + 1):
            b = g[r0 + s:r1 + s]
            scores[s] = float(np.dot(tpl, b) / (np.linalg.norm(tpl) * np.linalg.norm(b) + 1e-6))
        k = int(scores.argmax())
        if lo_s < k < hi_s:              # parabolic sub-pixel refinement
            y0, y1, y2 = scores[k - 1:k + 2]
            den = y0 - 2 * y1 + y2
            k = k + (0.5 * (y0 - y2) / den if den != 0 else 0.0)
        out.append((k, float(scores.max())))
    return np.array(out)


def fit_release(t, s, lo=0.8, hi=30.0):
    """Fit s = 1/2 a (t - t_r)^2 for t > t_r (0 before) on samples up to s<=hi."""
    idx = np.flatnonzero(s > hi)
    end = idx[0] if idx.size else len(s)
    tt, ss = t[:end], s[:end]
    first = np.flatnonzero(ss > lo)
    if first.size < 3:
        return None
    t_guess = tt[first[0]] - 2.0

    def model(x, a, tr):
        return 0.5 * a * np.clip(x - tr, 0, None) ** 2
    try:
        (a, tr), cov = optimize.curve_fit(model, tt, ss, p0=(1.0, t_guess), maxfev=20000)
    except RuntimeError:
        return None
    resid = ss - model(tt, a, tr)
    return {'t_r': float(tr), 'a_px_per_frame2': float(a), 'sd_t_r': float(math.sqrt(max(cov[1, 1], 0))),
            'rms_px': float(np.sqrt(np.mean(resid ** 2))), 'n_pts': int(len(tt)),
            't_first_lo': float(tt[first[0]])}


def release_fixed_a(t, s, a, lo=1.0, hi=15.0):
    """t_r = t - sqrt(2 s / a) on early-fall samples (monotonic part only), median."""
    ests = []
    peak = 0.0
    for ti, si in zip(t, s):
        if si < peak - 2.0:      # tracking lost (blur / payload left the window)
            break
        peak = max(peak, si)
        if lo <= si <= hi:
            ests.append(ti - math.sqrt(2 * si / a))
    if len(ests) < 2:
        return None, None, 0
    return float(np.median(ests)), float(np.std(ests)), len(ests)


def run():
    rows = []
    curves = {}
    for vid, (cprefix, label, trials) in VIDEOS.items():
        ev = json.load(open(f'{vid}_events.json'))['events']
        opens = periodic_opens(ev)
        horn = np.load(f'{vid}_horn.npy', mmap_mode='r')
        pcol = np.load(f'{cprefix}_pcol.npy', mmap_mode='r')
        W = pcol.shape[2]
        cols = (int(W * 0.2), int(W * 0.8))
        assert len(opens) == len(trials), (vid, [o['f_servo'] for o in opens])
        for e, trial in zip(opens, trials):
            fs = e['f_servo']
            t_s, f_cross = servo_onset(horn, fs)
            tr_sh = track_shift(pcol, fs - 10, 70, (fs - 70, fs - 15), cols)
            t_axis = np.arange(fs - 10, fs + 60, dtype=float)
            fit = fit_release(t_axis, tr_sh[:, 0])
            curves[f'{vid}:{trial}'] = {'t': (t_axis - t_s).tolist(), 's': tr_sh[:, 0].tolist(),
                                        'fit': fit, 't_s': t_s}
            row = {'video': vid, 'trial': trial, 'f_servo_cross': f_cross, 't_servo': t_s,
                   'f_slider': e['f_slider'], 'f_release_thr': e['f_release']}
            row['shift'] = tr_sh[:, 0].tolist()
            row['t_axis'] = t_axis.tolist()
            row['fit_free'] = fit
            if False:
                row.update({'t_release': fit['t_r'], 'D_ms': (fit['t_r'] - t_s) / FPS * 1e3,
                            'D_thr_ms': (e['f_release'] - f_cross) / FPS * 1e3,
                            'servo_to_slider_ms': (e['f_slider'] - t_s) / FPS * 1e3,
                            'slider_to_release_ms': (fit['t_r'] - e['f_slider']) / FPS * 1e3,
                            'a_px_f2': fit['a_px_per_frame2'], 'fit_rms_px': fit['rms_px'],
                            'fit_sd_ms': fit['sd_t_r'] / FPS * 1e3})
            rows.append(row)
    # acceleration per video from clean free fits (rms < 0.4 px)
    for vid in VIDEOS:
        good = [r['fit_free']['a_px_per_frame2'] for r in rows if r['video'] == vid and r['fit_free']
                and r['fit_free']['rms_px'] < 0.4 and r['fit_free']['a_px_per_frame2'] > 0.3]
        a_v = float(np.median(good)) if good else 0.75
        for r in rows:
            if r['video'] != vid:
                continue
            r['a_used'] = a_v
            r['n_clean_fits_video'] = len(good)
            t_r, sd, n_pts = release_fixed_a(np.array(r['t_axis']), np.array(r['shift']), a_v)
            r.update({'t_release': t_r, 'tr_spread_frames': sd, 'tr_points': n_pts})
            if t_r is not None and r['t_servo'] is not None:
                r['D_ms'] = (t_r - r['t_servo']) / FPS * 1e3
                r['servo_to_slider_ms'] = (r['f_slider'] - r['t_servo']) / FPS * 1e3
                r['slider_to_release_ms'] = (t_r - r['f_slider']) / FPS * 1e3
            r['D_thr_ms'] = (r['f_release_thr'] - r['f_servo_cross']) / FPS * 1e3
    json.dump({'rows': rows, 'curves': curves}, open('drop_delay_results.json', 'w'), indent=1, default=float)
    return rows, curves


if __name__ == '__main__':
    rows, _ = run()
    for r in rows:
        ff = r['fit_free']
        print(r['video'], r['trial'], 'D_ms', r.get('D_ms') and round(r['D_ms'], 1), 'D_thr', round(r['D_thr_ms'], 1),
              'spread_fr', r.get('tr_spread_frames') and round(r['tr_spread_frames'], 2), 'pts', r.get('tr_points'),
              'a_used', round(r['a_used'], 3), 'free_a', ff and round(ff['a_px_per_frame2'], 3), 'free_rms', ff and round(ff['rms_px'], 2),
              's:', [round(x, 1) for x in r['shift'][5:45]])

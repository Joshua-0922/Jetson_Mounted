"""Drop latency from a PX4 ulog (+ the drop node's session JSON).

    ros2 run payload_drop drop_latency log.ulg ~/drop_logs/drop_bench_<stamp>.json [--csv out.csv]
    ros2 run payload_drop drop_latency log.ulg            # ulog only (e.g. commands sent from QGC)

Stages, all on the PX4 boot clock (ulog timestamps):
  t0  PC send                 session JSON: PC time mapped with MAVLink TIMESYNC (+/- min RTT / 2)
  t1  PX4 received command    vehicle_command (DO_SET_ACTUATOR).timestamp
  t2  FMU output changed      actuator_outputs: first sample with the new pulse width
  t3  release sensor edge     camera_trigger.timestamp (timer-captured edge on the Camera Capture pin)
  link = t1-t0, px4 = t2-t1, release = t3-t2 (PWM frame <= 20 ms at 50 Hz + servo travel + latch),
  total = t3-t0. Optional --imu: first accelerometer disturbance after t2 (reference only).
"""
import argparse
import bisect
import csv
import json
import math
import statistics
import sys

CMD_DO_SET_ACTUATOR = 187


# ------------------------------------------------------------------------------ pure functions
def match_commands(events, cmd_t, cmd_val, tol_before_us=300e3, window_us=3e6):
    """Map each session event to the first not-yet-used ulog command with the same value whose
    time is within [est - tol_before, est + window]. Returns list of index or None."""
    used = set()
    out = []
    for ev in events:
        est = ev.get('px4_send_us')
        hit = None
        for i, (t, v) in enumerate(zip(cmd_t, cmd_val)):
            if i in used or not math.isclose(v, ev['value'], abs_tol=1e-3):
                continue
            if est is None or (est - tol_before_us <= t <= est + window_us):
                hit = i
                break
        if hit is not None:
            used.add(hit)
        out.append(hit)
    return out


def first_change(t, y, t_cmd, thr=20.0, window_us=500e3):
    """First sample at/after t_cmd whose value differs from the last value before t_cmd by > thr."""
    i = bisect.bisect_left(t, t_cmd)
    if i == 0 or i >= len(t):
        return None
    prev = y[i - 1]
    for k in range(i, len(t)):
        if t[k] - t_cmd > window_us:
            return None
        if abs(y[k] - prev) > thr:
            return t[k]
    return None


def first_after(ts, t0, window_us):
    i = bisect.bisect_left(ts, t0)
    if i < len(ts) and ts[i] - t0 <= window_us:
        return ts[i]
    return None


def imu_onset(t, a, t0, base_us=500e3, window_us=1e6, k=6.0, floor=0.5, hold=3):
    """First time after t0 where |a - mean| exceeds max(k*std, floor) for `hold` samples,
    baseline from the `base_us` before t0."""
    i0 = bisect.bisect_left(t, t0 - base_us)
    i1 = bisect.bisect_left(t, t0)
    base = a[i0:i1]
    if len(base) < 10:
        return None
    mu, sd = statistics.fmean(base), statistics.pstdev(base)
    thr = max(k * sd, floor)
    run = 0
    for j in range(i1, len(t)):
        if t[j] - t0 > window_us:
            return None
        run = run + 1 if abs(a[j] - mu) > thr else 0
        if run >= hold:
            return t[j - hold + 1]
    return None


def pick_output(outputs, cmd_times, channel_hint=None):
    """outputs: {instance: (t, [ch0, ch1, ...])}. Choose (instance, channel) that changes after the
    most commands. channel_hint (0-based) restricts the search to that channel."""
    best, best_n = None, 0
    for inst, (t, chans) in outputs.items():
        rng = [channel_hint] if channel_hint is not None else range(len(chans))
        for ch in rng:
            if ch >= len(chans):
                continue
            n = sum(first_change(t, chans[ch], tc) is not None for tc in cmd_times)
            if n > best_n:
                best, best_n = (inst, ch), n
    return best


def stats(values):
    v = sorted(x for x in values if x is not None)
    if not v:
        return None

    def q(p):
        return v[min(len(v) - 1, int(round(p * (len(v) - 1))))]
    return {'n': len(v), 'mean': statistics.fmean(v), 'std': statistics.pstdev(v) if len(v) > 1 else 0.0,
            'p50': q(0.5), 'p95': q(0.95), 'min': v[0], 'max': v[-1]}


def analyze(events, cmd_t, cmd_val, outputs, trig_t, actuator_set=1, channel_hint=None,
            imu=None, release_window_us=1.5e6):
    """Core analysis on plain lists (see load_ulog). Returns (rows, info)."""
    idx = match_commands(events, cmd_t, cmd_val)
    cmd_times = [cmd_t[i] for i in idx if i is not None]
    sel = pick_output(outputs, cmd_times, channel_hint)
    info = {'output': sel, 'n_events': len(events), 'n_matched': len(cmd_times)}
    if sel is not None:
        t_out = outputs[sel[0]][0]
        dts = [b - a for a, b in zip(t_out, t_out[1:])]
        info['output_dt_ms'] = statistics.median(dts) / 1e3 if dts else None
    rows = []
    for ev, i in zip(events, idx):
        r = {'seq': ev.get('seq'), 'kind': ev['kind'], 'value': ev['value'],
             'sync_unc_ms': None if ev.get('sync_unc_us') is None else ev['sync_unc_us'] / 1e3,
             'ack_rtt_ms': ev.get('ack_rtt_ms')}
        r['sends'] = ev.get('sends', 1)
        # resent (first copy lost or ACK lost): the ulog arrival may be the 2nd copy -> no link time
        t0 = ev.get('px4_send_us') if r['sends'] == 1 else None
        t1 = cmd_t[i] if i is not None else None
        t2 = t3 = t4 = None
        if t1 is not None and sel is not None:
            t, chans = outputs[sel[0]]
            t2 = first_change(t, chans[sel[1]], t1)
        if t2 is not None and ev['kind'] == 'open':
            t3 = first_after(trig_t, t2, release_window_us)
            if imu is not None:
                t4 = imu_onset(imu[0], imu[1], t2)
        ms = lambda a, b: None if a is None or b is None else (b - a) / 1e3   # noqa: E731
        r.update({'t_cmd_s': None if t1 is None else t1 / 1e6,
                  'link_ms': ms(t0, t1), 'px4_ms': ms(t1, t2), 'release_ms': ms(t2, t3),
                  'cmd_to_release_ms': ms(t1, t3), 'total_ms': ms(t0, t3),
                  'imu_after_output_ms': ms(t2, t4)})
        rows.append(r)
    return rows, info


# ------------------------------------------------------------------------------ ulog I/O
def load_ulog(path, actuator_set, with_imu):
    from pyulog import ULog
    names = ['vehicle_command', 'actuator_outputs', 'camera_trigger'] + (['sensor_combined'] if with_imu else [])
    ulog = ULog(path, names)
    cmd_t, cmd_val, outputs, trig_t, imu = [], [], {}, [], None
    for d in ulog.data_list:
        D = d.data
        if d.name == 'vehicle_command':
            for k in range(len(D['timestamp'])):
                if int(D['command'][k]) == CMD_DO_SET_ACTUATOR and round(float(D['param7'][k])) == 0:
                    cmd_t.append(int(D['timestamp'][k]))
                    cmd_val.append(float(D[f'param{actuator_set}'][k]))
        elif d.name == 'actuator_outputs':
            n = int(max(D['noutputs'])) if len(D['noutputs']) else 0
            outputs[d.multi_id] = ([int(x) for x in D['timestamp']],
                                   [[float(x) for x in D[f'output[{c}]']] for c in range(n)])
        elif d.name == 'camera_trigger':
            trig_t += [int(x) for x in D['timestamp']]
        elif d.name == 'sensor_combined':
            imu = ([int(x) for x in D['timestamp']], [float(x) for x in D['accelerometer_m_s2[2]']])
    order = sorted(range(len(cmd_t)), key=lambda i: cmd_t[i])
    return [cmd_t[i] for i in order], [cmd_val[i] for i in order], outputs, sorted(trig_t), imu


def events_from_ulog(cmd_t, cmd_val, open_value):
    return [{'seq': i, 'kind': 'open' if math.isclose(v, open_value, abs_tol=1e-3) else 'close',
             'value': v, 'px4_send_us': None} for i, v in enumerate(cmd_val)]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ulog')
    ap.add_argument('session', nargs='?', help='drop node session JSON (gives the PC send times)')
    ap.add_argument('--actuator-set', type=int, default=None)
    ap.add_argument('--open-value', type=float, default=1.0, help='ulog-only mode')
    ap.add_argument('--channel', type=int, default=None, help='FMU output channel (1-based) if not auto')
    ap.add_argument('--imu', action='store_true', help='also report accelerometer disturbance onset')
    ap.add_argument('--csv', help='write per-command rows')
    a = ap.parse_args(argv)

    sess = None
    if a.session:
        with open(a.session) as fh:
            sess = json.load(fh)
    aset = a.actuator_set or (sess or {}).get('actuator_set', 1)
    cmd_t, cmd_val, outputs, trig_t, imu = load_ulog(a.ulog, aset, a.imu)
    if sess:
        events = sess['events']
        ch = a.channel or ((sess.get('output') or {}).get('channel'))
    else:
        events = events_from_ulog(cmd_t, cmd_val, a.open_value)
        ch = a.channel
    rows, info = analyze(events, cmd_t, cmd_val, outputs, trig_t, aset,
                         None if ch is None else ch - 1, imu)

    print(f"ulog: {len(cmd_t)} DO_SET_ACTUATOR commands, {len(trig_t)} release-sensor edges, "
          f"actuator_outputs instances {sorted(outputs)}")
    print(f"matched {info['n_matched']}/{info['n_events']} commands; output = instance/channel "
          f"{info['output'] and (info['output'][0], info['output'][1] + 1)}, "
          f"sample spacing {info.get('output_dt_ms') or float('nan'):.1f} ms")
    if info.get('output_dt_ms') and info['output_dt_ms'] > 10:
        print('  WARNING: actuator_outputs is logged too slowly for this test: put '
              'config/logger_topics.txt on the SD card (see README)')
    if not trig_t:
        print('  no release-sensor edges (camera_trigger): release/total columns stay empty')
    cols = ['seq', 'kind', 'sends', 'link_ms', 'px4_ms', 'release_ms', 'cmd_to_release_ms', 'total_ms',
            'sync_unc_ms', 'ack_rtt_ms'] + (['imu_after_output_ms'] if a.imu else [])
    f = lambda v: '' if v is None else (f'{v:.1f}' if isinstance(v, float) else str(v))  # noqa: E731
    print('\n' + ' '.join(f'{c:>12}' for c in cols))
    for r in rows:
        print(' '.join(f'{f(r.get(c)):>12}' for c in cols))
    print('\nstage (drops only)        n    mean     std     p50     p95     min     max  [ms]')
    for c in cols[3:]:
        s = stats([r[c] for r in rows if r['kind'] == 'open'])
        if s:
            print(f"{c:<22} {s['n']:>4} {s['mean']:7.1f} {s['std']:7.1f} {s['p50']:7.1f} "
                  f"{s['p95']:7.1f} {s['min']:7.1f} {s['max']:7.1f}")
    if a.csv:
        with open(a.csv, 'w', newline='') as fh:
            w = csv.DictWriter(fh, fieldnames=list(rows[0]) if rows else cols)
            w.writeheader()
            w.writerows(rows)
        print(f'wrote {a.csv}')
    return 0


if __name__ == '__main__':
    sys.exit(main())

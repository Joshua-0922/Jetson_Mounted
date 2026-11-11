#!/usr/bin/env python3
"""Collect the measured sim2real numbers into CSV files under docs/sim2real_params/data.

    python3 scripts/build_sim2real_params.py [--drop-results PATH_TO_drop_delay_results.json]

Sources (all in measurements/): link_usb_*.json, *_tegrastats.log, *_yolo.json, yolo_latency_*.json,
drop_commands_*.jsonl, drop_delay_video_*_{trials,stats}.json. The optional full drop result file
(from scripts/drop_video/analyze_drop.py) adds the per-frame payload tracks.
"""
import argparse
import csv
import json
import os
import re
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
M = os.path.join(ROOT, 'measurements')
OUT = os.path.join(ROOT, 'docs', 'sim2real_params', 'data')
STAT_KEYS = ['n', 'mean', 'std', 'min', 'p50', 'p95', 'p99', 'max']


def w(name, header, rows):
    with open(os.path.join(OUT, name), 'w', newline='') as fh:
        cw = csv.writer(fh)
        cw.writerow(header)
        for r in rows:
            cw.writerow(['' if v is None else (round(v, 4) if isinstance(v, float) else v) for v in r])
    print('wrote', name, len(rows), 'rows')


def stat_row(prefix, metric, s):
    return prefix + [metric] + [s.get(k) for k in STAT_KEYS]


def link():
    rows = []
    for f in sorted(os.listdir(M)):
        m = re.match(r'link_usb_(\d+_\d+)(?:_([ABC]))?\.json$', f)
        if not m:
            continue
        d = json.load(open(os.path.join(M, f)))
        cond = m.group(2) or 'first'
        valid = 'no (uncorrected clock offset)' if cond == 'first' else 'yes'
        for k, v in d.items():
            if isinstance(v, dict) and 'mean' in v:
                rows.append(stat_row([m.group(1), cond, valid], k, v))
        rows.append([m.group(1), cond, valid, 'cmd_sent/acked/lost', d.get('cmd_sent'), None, None, None,
                     d.get('cmd_acked'), None, None, d.get('cmd_lost')])
    w('link_usb_xrce_20261028.csv', ['session', 'condition', 'valid', 'metric'] + STAT_KEYS, rows)


def tegrastats():
    rows = []
    pat = re.compile(r'(\d\d-\d\d-\d{4} \d\d:\d\d:\d\d) RAM (\d+)/(\d+)MB .*?CPU \[([^\]]+)\] GR3D_FREQ (\d+)%'
                     r'.*?cpu@([\d.]+)C.*?gpu@([\d.]+)C')
    for f in sorted(os.listdir(M)):
        m = re.match(r'link_usb_(\d+_\d+)_([ABC])_tegrastats\.log$', f)
        if not m:
            continue
        for line in open(os.path.join(M, f)):
            g = pat.search(line)
            if not g:
                continue
            cpus = [int(c.split('%')[0]) if '%' in c else None for c in g.group(4).split(',')]
            rows.append([m.group(2), g.group(1), int(g.group(2)), int(g.group(3))] + cpus[:6] +
                        [int(g.group(5)), float(g.group(6)), float(g.group(7))])
    w('jetson_tegrastats_20261028.csv', ['condition', 'time', 'ram_used_mb', 'ram_total_mb'] +
      [f'cpu{i}_pct' for i in range(6)] + ['gpu_pct', 'cpu_temp_c', 'gpu_temp_c'], rows)


def yolo_load():
    rows = []
    for f in sorted(os.listdir(M)):
        m = re.match(r'link_usb_(\d+_\d+)_([BC])_yolo\.json$', f)
        if not m:
            continue
        d = json.load(open(os.path.join(M, f)))
        for k in ('preprocess_ms', 'inference_ms', 'postprocess_ms', 'grab_to_result_ms'):
            rows.append(stat_row([m.group(2), round(d['achieved_fps'], 2)], k, d[k]))
    w('yolo11n_trt_load_20261028.csv', ['condition', 'achieved_fps', 'metric'] + STAT_KEYS, rows)


def camera_yolo():
    rows = []
    for f in sorted(os.listdir(M)):
        m = re.match(r'yolo_latency_(\d+_\d+)_([AB])\.json$', f)
        if not m:
            continue
        d = json.load(open(os.path.join(M, f)))
        pre = [m.group(2), 'publish_annotated=' + ('true' if m.group(2) == 'A' else 'false'),
               round(d['detection_rate_hz'], 2), round(d.get('skip_ratio', 0), 4)]
        rows.append(stat_row(pre, 'sof_to_detection_received', d['consumer_sof_to_detection_ms']))
        for k, v in (d.get('stages_ms') or {}).items():
            if v:
                rows.append(stat_row(pre, k, v))
        rows.append(stat_row(pre, 'inter_arrival', d['inter_arrival_ms']))
        for k, v in d.items():
            if k.startswith('held_age_ms') and isinstance(v, dict):
                rows.append(stat_row(pre, 'held_age_random_query', v))
    w('camera_yolo_latency_20261105.csv', ['condition', 'config', 'rate_hz', 'skip_ratio', 'metric'] + STAT_KEYS, rows)


def drop(results_path):
    cmds = [json.loads(l) for l in open(os.path.join(M, 'drop_commands_20261108.jsonl'))]
    rows = []
    for c in cmds:
        o, cl = c['open'], c['close']
        s = c.get('series') or {}
        rows.append([c['trial'], time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(o['t_send_wall'])) +
                     f".{int(o['t_send_wall'] % 1 * 1000):03d}", o['value'], o['ack_result'], o['ack_rtt_ms'],
                     o['sends'], cl['ack_rtt_ms'], c.get('hold_s', s.get('hold_s')), s.get('period_s'),
                     o['sync_unc_us'] / 1000 if o.get('sync_unc_us') else None])
    w('drop_radio_commands_20261108.csv',
      ['trial', 'open_sent_local_time', 'open_value', 'open_ack_result', 'open_ack_rtt_ms', 'open_sends',
       'close_ack_rtt_ms', 'hold_s', 'period_s', 'timesync_unc_ms'], rows)

    trials = json.load(open(os.path.join(M, 'drop_delay_video_20261108_trials.json')))
    st = json.load(open(os.path.join(M, 'drop_delay_video_20261108_stats.json')))
    rtt = {c['trial']: c['open']['ack_rtt_ms'] for c in cmds}
    out = {('v1', 8)}
    rows = [[t['video'], t['trial'], t['f_servo_cross'], t['t_servo'], t.get('t_release'), t.get('f_slider'),
             t.get('f_release_thr'), t.get('D_ms'), t.get('D_thr_ms'), t.get('servo_to_slider_ms'),
             t.get('slider_to_release_ms'), t.get('a_used'), rtt.get(t['trial']),
             int((t['video'], t['trial']) in out)] for t in trials]
    w('drop_delay_video_20261108.csv',
      ['video', 'trial', 'frame_servo_cross', 'frame_servo_onset', 'frame_release', 'frame_slider_onset',
       'frame_release_threshold', 'delay_servo_to_release_ms', 'delay_threshold_method_ms', 'servo_to_slider_ms',
       'slider_to_release_ms', 'fall_accel_px_per_frame2', 'radio_ack_rtt_ms', 'outlier'], rows)

    rows = []
    for k, v in st['fits_clean'].items():
        if isinstance(v, dict):
            rows.append([k] + [*(v['params'] + [None] * 3)[:3]] + [v['loglik'], v['AIC'], v['KS_D'], v['KS_p']])
    rows.append(['shapiro_normality_p', None, None, None, None, None, None, st['fits_clean']['shapiro_p']])
    w('drop_delay_fits_20261108.csv', ['distribution', 'param1', 'param2', 'param3', 'loglik', 'AIC', 'KS_D', 'KS_p'],
      rows)

    rows = [[k, v['n'], v['pearson_r'], v['pearson_p'], v['spearman_rho'], v['spearman_p']] for k, v in st['corr'].items()]
    rows.append(['v1_vs_v2_welch_t', None, st['v1_vs_v2_welch_t']['t'], st['v1_vs_v2_welch_t']['p'], None, None])
    rows.append(['v1_vs_v2_levene', None, None, st['v1_vs_v2_levene_p'], None, None])
    w('drop_delay_correlations_20261108.csv', ['pair', 'n', 'pearson_r', 'pearson_p', 'spearman_rho', 'spearman_p'], rows)

    rows = [[k, v['n'], v['mean'], v['std'], v['median'], v['min'], v['max'], v['p5'], v['p95'], v['sem']]
            for k, v in st.items() if isinstance(v, dict) and 'mean' in v]
    w('drop_delay_summary_20261108.csv', ['subset', 'n', 'mean_ms', 'std_ms', 'median_ms', 'min_ms', 'max_ms',
                                          'p5_ms', 'p95_ms', 'sem_ms'], rows)

    if results_path and os.path.exists(results_path):
        full = json.load(open(results_path))['rows']
        rows = []
        for r in full:
            for f, s in zip(r['t_axis'], r['shift']):
                rows.append([r['video'], r['trial'], int(f), round((f - r['t_servo']) / 240 * 1e3, 3), s])
        w('drop_video_payload_tracks_20261108.csv',
          ['video', 'trial', 'frame', 't_after_servo_onset_ms', 'payload_drop_px'], rows)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--drop-results')
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    link()
    tegrastats()
    yolo_load()
    camera_yolo()
    drop(a.drop_results)

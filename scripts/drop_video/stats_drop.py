"""Distribution fits, correlations and figure for the video drop delays."""
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

R = json.load(open('drop_delay_results.json'))['rows']
trials = {json.loads(l)['trial']: json.loads(l) for l in open(os.path.expanduser('~/drop_logs/video_trials.jsonl'))}
OUTLIER = {('v1', 8)}          # payload dropped 1-2 frames after servo onset: latch not engaged

for r in R:
    t = trials.get(r['trial'])
    r['ack_rtt_ms'] = t['open']['ack_rtt_ms'] if t else None
    r['outlier'] = (r['video'], r['trial']) in OUTLIER


def describe(x):
    x = np.asarray(x, float)
    return {'n': len(x), 'mean': x.mean(), 'std': x.std(ddof=1), 'median': float(np.median(x)),
            'min': x.min(), 'max': x.max(), 'p5': float(np.percentile(x, 5)), 'p95': float(np.percentile(x, 95)),
            'sem': x.std(ddof=1) / math.sqrt(len(x))}


def fits(x):
    x = np.asarray(x, float)
    out = {}
    cands = {
        'normal': (stats.norm, stats.norm.fit(x)),
        'uniform': (stats.uniform, (x.min() - 1e-9, x.max() - x.min() + 2e-9)),
        'lognormal': (stats.lognorm, stats.lognorm.fit(x, floc=0)),
        'gamma': (stats.gamma, stats.gamma.fit(x, floc=0)),
    }
    for name, (dist, p) in cands.items():
        ll = float(np.sum(dist.logpdf(x, *p)))
        k = 2
        ks = stats.kstest(x, dist.cdf, args=p)
        out[name] = {'params': [float(v) for v in p], 'loglik': ll, 'AIC': 2 * k - 2 * ll,
                     'KS_D': float(ks.statistic), 'KS_p': float(ks.pvalue)}
    out['shapiro_p'] = float(stats.shapiro(x).pvalue)
    return out


def corr(a, b):
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = ~(np.isnan(a) | np.isnan(b))
    pr = stats.pearsonr(a[m], b[m])
    sr = stats.spearmanr(a[m], b[m])
    return {'n': int(m.sum()), 'pearson_r': float(pr.statistic), 'pearson_p': float(pr.pvalue),
            'spearman_rho': float(sr.statistic), 'spearman_p': float(sr.pvalue)}


ok = [r for r in R if r.get('D_ms') is not None]
clean = [r for r in ok if not r['outlier']]
D_all = [r['D_ms'] for r in ok]
D = [r['D_ms'] for r in clean]
res = {
    'per_trial': [{k: r.get(k) for k in ('video', 'trial', 'D_ms', 'D_thr_ms', 'servo_to_slider_ms',
                                         'slider_to_release_ms', 'ack_rtt_ms', 'outlier')} for r in R],
    'all': describe(D_all), 'clean': describe(D), 'v1': describe([r['D_ms'] for r in clean if r['video'] == 'v1']),
    'v2': describe([r['D_ms'] for r in clean if r['video'] == 'v2']),
    'threshold_method_clean': describe([r['D_thr_ms'] for r in clean]),
    'fits_clean': fits(D),
    'corr': {
        'D_vs_trial_order': corr([i for i, _ in enumerate(clean)], D),
        'D_lag1_autocorr': corr(D[:-1], D[1:]),
        'servo_to_slider_vs_slider_to_release': corr([r['servo_to_slider_ms'] for r in clean],
                                                      [r['slider_to_release_ms'] for r in clean]),
        'D_vs_servo_to_slider': corr(D, [r['servo_to_slider_ms'] for r in clean]),
        'D_vs_ack_rtt': corr(D, [r['ack_rtt_ms'] for r in clean]),
        'D_vs_threshold_method': corr(D, [r['D_thr_ms'] for r in clean]),
    },
    'v1_vs_v2_welch_t': {k: float(v) for k, v in zip(('t', 'p'), stats.ttest_ind(
        [r['D_ms'] for r in clean if r['video'] == 'v1'], [r['D_ms'] for r in clean if r['video'] == 'v2'],
        equal_var=False))},
    'v1_vs_v2_levene_p': float(stats.levene([r['D_ms'] for r in clean if r['video'] == 'v1'],
                                            [r['D_ms'] for r in clean if r['video'] == 'v2']).pvalue),
}
json.dump(res, open('drop_delay_stats.json', 'w'), indent=1, default=float)

# ---------------- figure
fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))
x = np.asarray(D)
bins = np.arange(30, 100, 5)
ax[0].hist([r['D_ms'] for r in clean if r['video'] == 'v1'], bins=bins, alpha=0.7, label='video 1 (trials 3-12)', color='#4C72B0')
ax[0].hist([r['D_ms'] for r in clean if r['video'] == 'v2'], bins=bins, alpha=0.7, label='video 2 (trials 17-26)', color='#DD8452')
xx = np.linspace(20, 110, 300)
f = res['fits_clean']
scale = len(x) * 5
ax[0].plot(xx, scale * stats.norm.pdf(xx, *f['normal']['params']), 'k-', label=f"N({f['normal']['params'][0]:.1f}, {f['normal']['params'][1]:.1f})")
ax[0].plot(xx, scale * stats.uniform.pdf(xx, *f['uniform']['params']), 'k--', label=f"U({x.min():.1f}, {x.max():.1f})")
ax[0].plot(xx, scale * stats.lognorm.pdf(xx, *f['lognormal']['params']), 'g:', label='lognormal')
ax[0].set_xlabel('servo motion onset -> payload release [ms]'); ax[0].set_ylabel('count'); ax[0].legend(fontsize=8)
ax[0].set_title(f'Drop delay, n={len(x)} (trial 8 excluded)')

for r in R:
    c = '#4C72B0' if r['video'] == 'v1' else '#DD8452'
    mk = 'x' if r['outlier'] else 'o'
    if r.get('D_ms') is not None:
        ax[1].plot(r['trial'], r['D_ms'], mk, color=c, ms=7)
        ax[1].plot(r['trial'], r['D_thr_ms'], '.', color='gray', ms=5)
ax[1].axhline(np.mean(x), color='k', lw=0.8)
ax[1].axhspan(np.mean(x) - np.std(x, ddof=1), np.mean(x) + np.std(x, ddof=1), color='k', alpha=0.08)
ax[1].set_xlabel('trial'); ax[1].set_ylabel('ms'); ax[1].set_title('per trial (o fall-fit, . threshold, x outlier)')

for r in R:
    t = (np.array(r['t_axis']) - r['t_servo']) / 240 * 1e3
    s = np.array(r['shift'])
    c = '#4C72B0' if r['video'] == 'v1' else '#DD8452'
    ax[2].plot(t, s, '-', color=c, alpha=0.5 if not r['outlier'] else 0.9, lw=1)
ax[2].set_xlim(-20, 160); ax[2].set_ylim(-2, 72)
ax[2].set_xlabel('time after servo onset [ms]'); ax[2].set_ylabel('payload drop [px]')
ax[2].set_title('tracked payload fall (free fall = parabola)')
plt.tight_layout()
plt.savefig('drop_delay.png', dpi=110)

print(json.dumps({k: res[k] for k in ('all', 'clean', 'v1', 'v2', 'threshold_method_clean')}, indent=1, default=lambda v: round(float(v), 2)))
print(json.dumps(res['fits_clean'], indent=1, default=lambda v: round(float(v), 3)))
print(json.dumps(res['corr'], indent=1, default=lambda v: round(float(v), 3)))
print('v1 vs v2 welch', res['v1_vs_v2_welch_t'], 'levene p', res['v1_vs_v2_levene_p'])
for r in res['per_trial']:
    print(r)

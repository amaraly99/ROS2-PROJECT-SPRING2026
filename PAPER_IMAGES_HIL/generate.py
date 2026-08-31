#!/usr/bin/env python3
"""Regenerate tables and figures for the SLAM ablation section.

Two evaluations, same shape:
  offline  EuRoC playback, from results/.../experiment_summary.csv
  online   simulation sweep, from the recorded bags

Accuracy is reported as MEDIAN APE RMSE in metres. Reliability is the fraction
of attempted trials that completed and tracked. Nothing is typed by hand.

Usage:  python3 PAPER_IMAGES_HIL/generate.py
"""
import csv
import glob
import importlib.util
import itertools
import json
import statistics as st
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
HOME = REPO.parent

spec = importlib.util.spec_from_file_location(
    'agg', REPO / 'scripts/hil_matrix/aggregate_matrix.py')
agg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agg)

END_PAD = 3

# ---- offline: EuRoC playback -----------------------------------------
# ORB-SLAM2 has no EuRoC experiment_summary.csv anywhere in the project, so it
# is absent from the offline comparison. Stated rather than substituted.
OFFLINE = [
    ('ORB-SLAM2',
     'ORB_SLAM2/results/20260609_092931/experiment_summary.csv'),
    ('ORB-SLAM3',
     'ORBSLAM3_ROS2/results/orbslam_benchmark/20260614_014256/experiment_summary.csv'),
    ('OV2SLAM-acc',
     'ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_v2/20260612_234141/accurate_mono/experiment_summary.csv'),
    ('OV2SLAM-fast',
     'ROS2-PROJECT-SPRING2026/results/ov2slam_benchmark_v2/20260613_102424/fast_mono/experiment_summary.csv'),
]

# ---- online: simulation sweep ----------------------------------------
ONLINE = [
    ('ORB-SLAM2',    'bags/run_orbslam2_probe_verso10_*'),
    # Two complete 10-trial sets, pooled: statistically indistinguishable
    # (permutation p=0.077, identical sd), so n=20 gives a far more stable
    # median than either alone. Only orb3lat carries front-end timing, so the
    # latency figure comes from those 10 while accuracy uses all 20.
    # Deliberately NOT a bare orb3* glob: that also matches the aborted
    # init-only bags from earlier attempts, which inflate n and corrupt
    # reliability.
    ('ORB-SLAM3',    'bags/run_orbslam3_probe_orb3x10_*'),
    ('ORB-SLAM3',    'bags/run_orbslam3_probe_orb3lat_*'),
    ('OV2SLAM-acc',  'bags/run_ov2slam_oracle_accurate_verso10_*'),
    ('OV2SLAM-fast', 'bags/run_ov2slam_oracle_fast_verso10_*'),
]

ORDER = ['ORB-SLAM2', 'ORB-SLAM3', 'OV2SLAM-acc', 'OV2SLAM-fast', 'RTAB-Map']
COLOR = {'ORB-SLAM2': '#1f77b4', 'ORB-SLAM3': '#d62728',
         'OV2SLAM-acc': '#2ca02c', 'OV2SLAM-fast': '#ff7f0e',
         'RTAB-Map': '#9467bd'}

# RTAB-Map appears in the offline evaluation only. rtabmap_odom ships
# rgbd/stereo/icp odometry and no monocular node, so it cannot enter the
# monocular simulation sweep.
RTAB_SEL = 'ORB_SLAM2/images/trajectory_selection_summary.csv'


def f(row, key):
    v = (row.get(key) or '').strip()
    try:
        return float(v)
    except ValueError:
        return None


def load_offline():
    out = {}
    for name, rel in OFFLINE:
        p = HOME / rel
        if not p.exists():
            continue
        rows = list(csv.DictReader(open(p)))
        if not rows:
            continue

        # ORB-SLAM2 exports one PRE-AGGREGATED row per sequence (n_runs_total,
        # rmse_avg_all, ...) instead of one row per run, so it needs its own
        # reader. Same underlying design: 10 sequences x 10 runs.
        if 'n_runs_total' in rows[0]:
            tot = sum(int(r['n_runs_total']) for r in rows)
            comp = sum(int(r['n_runs_completed']) for r in rows)
            rm = [f(r, 'rmse_avg_all') for r in rows if f(r, 'rmse_avg_all') is not None]
            cp = [f(r, 'cpu_total_avg_pct') for r in rows
                  if f(r, 'cpu_total_avg_pct') is not None]
            lt = [f(r, 'frontend_full_tracking_avg_all_ms') for r in rows
                  if f(r, 'frontend_full_tracking_avg_all_ms') is not None]
            out[name] = dict(
                attempted=tot, valid=comp, reliability=100.0 * comp / tot,
                rmse=np.array(rm), rmse_med=float(st.median(rm)),
                cpu=float(st.mean(cp)) if cp else None,
                lat=float(st.mean(lt)) if lt else None,
                seqs=len(rows), src=rel)
            continue

        ok = [r for r in rows
              if (r.get('success') or '').strip() == 'True' and f(r, 'rmse') is not None]
        rmse = [f(r, 'rmse') for r in ok]
        cpu = [f(r, 'cpu_total_avg_pct') for r in ok if f(r, 'cpu_total_avg_pct') is not None]
        lat = [f(r, 'primary_track_mean_ms') for r in ok
               if f(r, 'primary_track_mean_ms') is not None]
        out[name] = dict(
            attempted=len(rows), valid=len(ok),
            reliability=100.0 * len(ok) / len(rows),
            rmse=np.array(rmse), rmse_med=float(st.median(rmse)) if rmse else float('nan'),
            cpu=float(st.mean(cpu)) if cpu else None,
            lat=float(st.mean(lat)) if lat else None,
            seqs=len(set(r['sequence'] for r in rows)),
            src=rel)

    # RTAB-Map is deliberately NOT reported in the results tables.
    #
    # The only EuRoC record for it is trajectory_selection_summary.csv, which is
    # a BEST-RUN-PER-SEQUENCE table: its selection_reason column is literally
    # "coverage>=90_lowest_rmse" (with a highest_coverage_fallback), and
    # selected_run varies over {1,2,3}. It was built to choose representative
    # trajectory PLOTS, not to report statistics.
    #
    # Including it would put a best-of-3 figure beside backends reported over all
    # 100 runs, which is optimistically biased and not comparable. RTAB-Map stays
    # in the candidate list with its exclusion reason (no monocular odometry
    # node) and out of the numbers.
    return out


def load_online():
    out = {}
    # A candidate may list several globs (pooled run sets); accumulate them.
    pats = {}
    for name, pat in ONLINE:
        pats.setdefault(name, []).append(pat)
    for name, plist in pats.items():
        bags = sorted(b for p in plist for b in glob.glob(str(REPO / p)))
        rows = []
        for b in bags:
            try:
                r = agg.evaluate(b, END_PAD)
                if 'error' not in r:
                    rows.append(r)
            except Exception:
                pass
        if not rows:
            continue
        rmse = np.array([r['ape_rmse'] for r in rows])
        def arr(k):
            v = [r[k] for r in rows if k in r and r[k] == r[k]]
            return np.array(v, float) if v else np.array([])
        def m(k):
            a = arr(k)
            return float(a.mean()) if a.size else None
        lat_a, cpu_a = arr('fe_ms_mean'), arr('cpu_mean_pct')
        out[name] = dict(
            attempted=len(bags), valid=len(rows),
            reliability=100.0 * len(rows) / max(len(bags), 1),
            rmse=rmse, rmse_med=float(np.median(rmse)),
            lat=m('fe_ms_mean'), cpu=m('cpu_mean_pct'), path=m('gt_path_m'),
            # per-run arrays so the bar charts can show real dispersion rather
            # than a mean with no error bar
            lat_arr=lat_a, cpu_arr=cpu_a,
            # front-end throughput: invert PER RUN, then aggregate. Inverting the
            # mean latency instead would give a different and wrong number.
            hz_arr=(1000.0 / lat_a) if lat_a.size else np.array([]),
            bags=[Path(b).name for b in bags])
    return out


def perm_p(a, b, it=100000, seed=0):
    rng = np.random.default_rng(seed)
    obs = abs(np.median(a) - np.median(b))
    pool = np.concatenate([a, b]); na = len(a); c = 0
    for _ in range(it):
        rng.shuffle(pool)
        if abs(np.median(pool[:na]) - np.median(pool[na:])) >= obs - 1e-12:
            c += 1
    return (c + 1) / (it + 1)


def holm(ps):
    order = np.argsort(ps); adj = [0.0] * len(ps); run = 0.0
    for i, idx in enumerate(order):
        run = max(run, min(1.0, ps[idx] * (len(ps) - i)))
        adj[idx] = run
    return adj


def w(path, lines):
    (HERE / path).write_text('\n'.join(lines) + '\n')


def num(x, fmt='%.3f'):
    return (fmt % x) if x is not None and x == x else r'n/a'


def main():
    OFF = load_offline()
    ON = load_online()
    if not ON:
        sys.exit('no online bags matched')

    # ---- T1 offline ---------------------------------------------------
    L = [r'\begin{tabular}{lrrrrr}', r'\toprule',
         r'Candidate & runs & reliability [\%] & median APE [m] & '
         r'latency [ms] & CPU [\%] \\', r'\midrule']
    for n in ORDER:
        if n not in OFF:
            continue
        d = OFF[n]
        L.append(r'%s & %d & %.1f & %.4f & %s & %s \\' % (
            n, d['attempted'], d['reliability'], d['rmse_med'],
            num(d['lat'], '%.1f'), num(d['cpu'], '%.1f')))
    L += [r'\bottomrule', r'\end{tabular}']
    w('tables/t1_offline.tex', L)

    # ---- T2 online ----------------------------------------------------
    L = [r'\begin{tabular}{lrrrrr}', r'\toprule',
         r'Candidate & runs & reliability [\%] & median APE [m] & '
         r'latency [ms] & CPU [\%] \\', r'\midrule']
    for n in ORDER:
        if n not in ON:
            continue
        d = ON[n]
        L.append(r'%s & %d & %.1f & %.4f & %s & %s \\' % (
            n, d['attempted'], d['reliability'], d['rmse_med'],
            num(d['lat'], '%.1f'), num(d['cpu'], '%.1f')))
    L += [r'\bottomrule', r'\end{tabular}']
    w('tables/t2_online.tex', L)

    # ---- T3 significance (online, median APE) --------------------------
    names = [n for n in ORDER if n in ON and len(ON[n]['rmse']) >= 3]
    pairs = list(itertools.combinations(names, 2))
    raw = [perm_p(ON[a]['rmse'], ON[b]['rmse']) for a, b in pairs]
    adj = holm(raw)
    L = [r'\begin{tabular}{llrrl}', r'\toprule',
         r'A & B & $\Delta$ median APE [m] & $p_{\mathrm{Holm}}$ & \\', r'\midrule']
    for (a, b), h in zip(pairs, adj):
        d = ON[a]['rmse_med'] - ON[b]['rmse_med']
        L.append(r'%s & %s & $%+.4f$ & %.4f & %s \\' % (
            a, b, d, h, 'differ' if h < 0.05 else 'n.s.'))
    L += [r'\bottomrule', r'\end{tabular}']
    w('tables/t3_significance.tex', L)

    def barplot(fname, series, ylab, title=None, note=None, fmt='%.2f'):
        """One bar per candidate with a standard-deviation whisker."""
        ns = [n for n, v in series if v is not None and len(v)]
        if not ns:
            return
        vals = dict(series)
        mu = [float(np.mean(vals[n])) for n in ns]
        sd = [float(np.std(vals[n], ddof=1)) if len(vals[n]) > 1 else 0.0 for n in ns]
        fig, ax = plt.subplots(figsize=(6.6, 3.5))
        ax.bar(range(len(ns)), mu, yerr=sd, capsize=4,
               color=[COLOR[n] for n in ns], edgecolor='k', linewidth=.4)
        for i, (m_, s_) in enumerate(zip(mu, sd)):
            ax.text(i, m_ + s_, fmt % m_, ha='center', va='bottom', fontsize=8)
        ax.set_xticks(range(len(ns)))
        ax.set_xticklabels(ns, rotation=12, fontsize=8)
        ax.set_ylabel(ylab, fontsize=9)
        if title:
            ax.set_title(title, fontsize=9)
        if note:
            ax.text(0.99, 0.97, note, transform=ax.transAxes, ha='right',
                    va='top', fontsize=7, style='italic', color='#444')
        ax.grid(alpha=.3, axis='y')
        fig.tight_layout(); fig.savefig(HERE / fname); plt.close(fig)

    # ---- F1 accuracy, mean +/- sd --------------------------------------
    fig, ax = plt.subplots(1, 2, figsize=(9.4, 3.5))
    for k, (D, ttl) in enumerate([(OFF, 'Offline (EuRoC playback)'),
                                  (ON, 'Online (simulation sweep)')]):
        ns = [n for n in ORDER if n in D]
        mu = [float(np.mean(D[n]['rmse'])) for n in ns]
        sd = [float(np.std(D[n]['rmse'], ddof=1)) if len(D[n]['rmse']) > 1 else 0.0
              for n in ns]
        ax[k].bar(range(len(ns)), mu, yerr=sd, capsize=4,
                  color=[COLOR[n] for n in ns], edgecolor='k', linewidth=.4)
        for i, (m_, s_) in enumerate(zip(mu, sd)):
            ax[k].text(i, m_ + s_, '%.3f' % m_, ha='center', va='bottom', fontsize=8)
        ax[k].set_xticks(range(len(ns)))
        ax[k].set_xticklabels(ns, rotation=12, fontsize=8)
        ax[k].set_title(ttl, fontsize=9); ax[k].grid(alpha=.3, axis='y')
    ax[0].set_ylabel('APE RMSE [m], mean $\\pm$ sd')
    fig.tight_layout(); fig.savefig(HERE / 'figures/f1_accuracy_mean.pdf'); plt.close(fig)

    # ---- F1b accuracy, median ------------------------------------------
    fig, ax = plt.subplots(1, 2, figsize=(9.4, 3.5))
    for k, (D, ttl) in enumerate([(OFF, 'Offline (EuRoC playback)'),
                                  (ON, 'Online (simulation sweep)')]):
        ns = [n for n in ORDER if n in D]
        med = [D[n]['rmse_med'] for n in ns]
        lo = [D[n]['rmse_med'] - np.percentile(D[n]['rmse'], 25) for n in ns]
        hi = [np.percentile(D[n]['rmse'], 75) - D[n]['rmse_med'] for n in ns]
        ax[k].bar(range(len(ns)), med, yerr=[lo, hi], capsize=4,
                  color=[COLOR[n] for n in ns], edgecolor='k', linewidth=.4)
        for i, m_ in enumerate(med):
            ax[k].text(i, m_ + hi[i], '%.3f' % m_, ha='center', va='bottom', fontsize=8)
        ax[k].set_xticks(range(len(ns)))
        ax[k].set_xticklabels(ns, rotation=12, fontsize=8)
        ax[k].set_title(ttl, fontsize=9); ax[k].grid(alpha=.3, axis='y')
    ax[0].set_ylabel('APE RMSE [m], median with IQR')
    fig.tight_layout(); fig.savefig(HERE / 'figures/f1b_accuracy_median.pdf'); plt.close(fig)

    # ---- F5 CPU, F6 latency, F7 throughput (online) ---------------------
    barplot('figures/f5_cpu.pdf',
            [(n, ON[n]['cpu_arr']) for n in ORDER if n in ON],
            'CPU utilisation [%], mean $\\pm$ sd',
            'Online sweep',
            'SLAM pinned to 2 of 4 cores: ceiling is 200%', '%.1f')
    barplot('figures/f6_latency.pdf',
            [(n, ON[n]['lat_arr']) for n in ORDER if n in ON],
            'front-end latency [ms], mean $\\pm$ sd',
            'Online sweep', 'per frame, 2-core allocation', '%.1f')
    barplot('figures/f7_throughput.pdf',
            [(n, ON[n]['hz_arr']) for n in ORDER if n in ON],
            'front-end throughput [Hz], mean $\\pm$ sd',
            'Online sweep',
            'sustainable rate; delivered stream is 13 Hz', '%.0f')

    # ---- F2 reliability -------------------------------------------------
    fig, ax = plt.subplots(figsize=(6.4, 3.2))
    ns = [n for n in ORDER if n in ON or n in OFF]
    x = np.arange(len(ns)); wd = 0.38
    ax.bar(x - wd/2, [OFF[n]['reliability'] if n in OFF else 0 for n in ns],
           wd, label='offline', color='#888')
    ax.bar(x + wd/2, [ON[n]['reliability'] if n in ON else 0 for n in ns],
           wd, label='online', color='#333')
    ax.set_xticks(x); ax.set_xticklabels(ns, rotation=15, fontsize=8)
    ax.set_ylabel('reliability [%]'); ax.set_ylim(0, 105)
    ax.axhline(100, ls=':', c='k', lw=.8)
    ax.legend(fontsize=8); ax.grid(alpha=.3, axis='y')
    fig.tight_layout(); fig.savefig(HERE / 'figures/f2_reliability.pdf'); plt.close(fig)

    # ---- F3 cost profile ------------------------------------------------
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.2))
    ns = [n for n in ORDER if n in ON]
    for k, (key, lab) in enumerate([('lat', 'front-end latency [ms]'),
                                    ('cpu', 'CPU utilisation [\\%]')]):
        vals = [ON[n][key] if ON[n][key] is not None else 0 for n in ns]
        ax[k].bar(range(len(ns)), vals, color=[COLOR[n] for n in ns])
        for i, (n, v) in enumerate(zip(ns, vals)):
            ax[k].text(i, v, 'n/a' if ON[n][key] is None else '%.1f' % v,
                       ha='center', va='bottom', fontsize=8)
        ax[k].set_xticks(range(len(ns)))
        ax[k].set_xticklabels(ns, rotation=15, fontsize=8)
        ax[k].set_ylabel(lab, fontsize=9)
        ax[k].grid(alpha=.3, axis='y')
    fig.tight_layout(); fig.savefig(HERE / 'figures/f3_cost.pdf'); plt.close(fig)

    # ---- F4 Pareto, median APE on the accuracy axis ---------------------
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.6))
    for k, (key, lab) in enumerate([('lat', 'front-end latency [ms]'),
                                    ('cpu', 'CPU utilisation [\\%]')]):
        for n in ORDER:
            if n not in ON or ON[n][key] is None:
                continue
            d = ON[n]
            q1, q3 = np.percentile(d['rmse'], [25, 75])
            ax[k].errorbar(d[key], d['rmse_med'],
                           yerr=[[d['rmse_med']-q1], [q3-d['rmse_med']]],
                           fmt='o', ms=7, capsize=3, color=COLOR[n])
            ax[k].annotate(n, (d[key], d['rmse_med']), textcoords='offset points',
                           xytext=(7, 4), fontsize=8)
        ax[k].set_xlabel(lab); ax[k].grid(alpha=.3)
    ax[0].set_ylabel('median APE RMSE [m]')
    fig.suptitle('Accuracy versus cost (lower left dominates)', fontsize=10)
    fig.tight_layout(); fig.savefig(HERE / 'figures/f4_pareto.pdf'); plt.close(fig)

    prov = dict(offline={n: dict(src=d['src'], attempted=d['attempted'],
                                 valid=d['valid'], seqs=d['seqs'])
                         for n, d in OFF.items()},
                online={n: dict(attempted=d['attempted'], valid=d['valid'],
                                bags=d['bags']) for n, d in ON.items()})
    (HERE / 'data/provenance.json').write_text(json.dumps(prov, indent=1))

    print('OFFLINE (EuRoC)')
    for n in ORDER:
        if n in OFF:
            d = OFF[n]
            print('  %-13s %3d runs  rel %5.1f%%  medAPE %.4f m  cpu %s  lat %s'
                  % (n, d['attempted'], d['reliability'], d['rmse_med'],
                     num(d['cpu'], '%.1f'), num(d['lat'], '%.1f')))
    print('ONLINE (simulation)')
    for n in ORDER:
        if n in ON:
            d = ON[n]
            print('  %-13s %3d runs  rel %5.1f%%  medAPE %.4f m  cpu %s  lat %s'
                  % (n, d['attempted'], d['reliability'], d['rmse_med'],
                     num(d['cpu'], '%.1f'), num(d['lat'], '%.1f')))


if __name__ == '__main__':
    main()

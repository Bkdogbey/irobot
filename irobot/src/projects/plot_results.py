"""
Publication figures for the pDSTL Crazyflie experiment.

Generates four figures:
  fig1_paths.pdf      — planned paths + 2-sigma covariance ellipses (before/after)
  fig2_experiment.pdf — trajectories at min/max wind + safety rate bar chart
  fig3_clearance.pdf  — minimum obstacle clearance vs fan speed (mean ± std across runs)
  fig4_all_speeds.pdf — mean paths at all fan speeds overlaid, one panel per condition

Experiment protocol: 4 fan speeds (0, 6, 12, 18) x 2 conditions (det, pDSTL) x 20 runs.
All statistics are computed across the 20 runs per cell.

Run from the repo root:
    python irobot/src/projects/plot_results.py
"""

from __future__ import annotations

import csv
import pathlib
import re
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, FancyArrowPatch, Rectangle

# ── Paths ─────────────────────────────────────────────────────────────────────
_HERE = pathlib.Path(__file__).parent
_STL_DIR = _HERE / 'crazyflie_stl'
_LOGS = _HERE / 'probabilistic_stl' / 'components' / 'logs'
_OPT_WP = _HERE / 'probabilistic_stl' / 'components' / 'opt_waypoints.py'
_OUT = _HERE / 'figures'
_OUT.mkdir(exist_ok=True)

sys.path.insert(0, str(_STL_DIR))
from belief import GaussianBelief2D  # noqa: E402
from stl import OutsideObstacle, stl_and  # noqa: E402

# ── Constants (must match optimise_cf_path.py and flight_logger.py) ───────────
START = np.array([0.0, -1.5])
GOAL = np.array([0.489, 0.65])
N_WP = 10
OBSTACLES = [
    (-0.25, 0.25, -1.05, -0.85),  # OBS-1
    (-0.45,  0.05, 0.10, 0.35),   # OBS-2
    ( 0.20,  0.55, -0.25, 0.10),  # OBS-3
]
SIGMA0 = np.eye(2) * (0.010**2)
SIGMA_TOTAL_STEP = float(np.sqrt(0.010**2 + 0.020**2))
DT = 0.1
ALPHA = 0.90
FWD_CUTOFF = 7.0


# =============================================================================
# Data loading — multi-run aware
# =============================================================================

def _build_fan_map() -> tuple[dict[tuple[int, str], list[pathlib.Path]], list[int]]:
    """
    Auto-scan _LOGS for actual CSVs produced by the new naming scheme:
        <condition>_fan<XX>_run<NN>_<ts>_actual.csv

    Crashed files (<condition>_fan<XX>_run<NN>_CRASH_<ts>_actual.csv) are excluded.

    Returns:
        fan_map  : {(fan_speed, path_type): [list of matching Path objects]}
        speeds   : sorted list of unique fan speeds found
    """
    pattern = re.compile(
        r'^(?P<cond>deterministic|pdstl)_fan(?P<speed>\d+)_run(?P<run>\d+)'
        r'_(?P<ts>\d{8}T\d{6})_actual\.csv$'
    )
    fan_map: dict[tuple[int, str], list[pathlib.Path]] = {}
    for f in sorted(_LOGS.glob('*_actual.csv')):
        m = pattern.match(f.name)
        if not m:
            continue  # old-format files, CRASH files, or unrelated
        cond = m.group('cond')
        speed = int(m.group('speed'))
        path_type = 'det' if cond == 'deterministic' else 'pdstl'
        key = (speed, path_type)
        fan_map.setdefault(key, []).append(f)

    speeds = sorted({k[0] for k in fan_map})
    return fan_map, speeds


FAN_MAP, FAN_SPEEDS = _build_fan_map()
N_LEVELS = len(FAN_SPEEDS)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _is_safe(x: float, y: float) -> bool:
    """Recompute safety against current OBSTACLES (overrides stale CSV flags)."""
    for x0, x1, y0, y1 in OBSTACLES:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return False
    return True


def load_csv(path: pathlib.Path) -> list[dict]:
    with open(path) as f:
        rows = [{k: (float(v) if k != 'condition' else v) for k, v in r.items()} for r in csv.DictReader(f)]
    # Recompute 'safe' using current OBSTACLES so stale CSV flags don't affect analysis
    for r in rows:
        r['safe'] = int(_is_safe(r['x'], r['y']))
    return rows


def load_all_runs(
    fan_map: dict[tuple[int, str], list[pathlib.Path]]
) -> dict[tuple[int, str], list[list[dict]]]:
    """
    Load every CSV for each (speed, path_type) cell.

    Returns:
        {(speed, path_type): [[rows_run1], [rows_run2], ...]}
        Each inner list contains only rows with t <= FWD_CUTOFF.
    """
    result: dict[tuple[int, str], list[list[dict]]] = {}
    for key, paths in fan_map.items():
        runs = []
        for p in paths:
            rows = [r for r in load_csv(p) if r['t'] <= FWD_CUTOFF]
            if rows:
                runs.append(rows)
        if runs:
            result[key] = runs
    return result


def _min_obs_clearance(x: float, y: float) -> float:
    """Minimum Euclidean clearance from point (x,y) to any obstacle boundary (cm)."""
    dists = []
    for x0, x1, y0, y1 in OBSTACLES:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        dists.append(float(np.hypot(dx, dy)) * 100.0)
    return min(dists)


def _run_stats(runs: list[list[dict]], planned_xy: np.ndarray) -> dict:
    """
    Compute per-run metrics and their mean ± std across runs.

    Per-run metrics:
        safety_rate  : fraction of safe samples * 100 (%)
        min_clr      : minimum obstacle clearance across all samples (cm)
        mean_dev     : mean nearest-waypoint deviation across samples (cm)

    Returns dict with keys:
        n_runs, sr_mean, sr_std, clr_mean, clr_std, dev_mean, dev_std,
        n_samples_total, viols_total
    """
    sr_list, clr_list, dev_list = [], [], []
    n_total = 0
    viols_total = 0

    for rows in runs:
        n = len(rows)
        viols = sum(1 for r in rows if not r['safe'])
        sr_list.append((n - viols) / n * 100.0)
        clr_list.append(min(_min_obs_clearance(r['x'], r['y']) for r in rows))
        devs = [
            min(float(np.hypot(r['x'] - px, r['y'] - py)) for px, py in planned_xy) * 100.0
            for r in rows
        ]
        dev_list.append(sum(devs) / len(devs))
        n_total += n
        viols_total += viols

    arr_sr  = np.array(sr_list)
    arr_clr = np.array(clr_list)
    arr_dev = np.array(dev_list)

    return {
        'n_runs':        len(runs),
        'sr_mean':       float(arr_sr.mean()),
        'sr_std':        float(arr_sr.std(ddof=1)) if len(runs) > 1 else 0.0,
        'clr_mean':      float(arr_clr.mean()),
        'clr_std':       float(arr_clr.std(ddof=1)) if len(runs) > 1 else 0.0,
        'dev_mean':      float(arr_dev.mean()),
        'dev_std':       float(arr_dev.std(ddof=1)) if len(runs) > 1 else 0.0,
        'n_samples':     n_total,
        'viols_total':   viols_total,
    }


def sine_waypoints() -> np.ndarray:
    y = np.linspace(-1.5, 0.65, N_WP)
    x = 0.5 * np.sin(np.pi * y / 1.5)
    return np.stack([x, y], axis=1)


def propagate(waypoints: np.ndarray) -> list[GaussianBelief2D]:
    q_std = SIGMA_TOTAL_STEP / DT
    beliefs = [GaussianBelief2D(mean=waypoints[0].copy(), cov=SIGMA0.copy())]
    for i in range(1, len(waypoints)):
        vel = (waypoints[i] - waypoints[i - 1]) / DT
        beliefs.append(beliefs[-1].predict(vel, DT, q_std=q_std))
    return beliefs


def opt_waypoints() -> np.ndarray:
    ns = {}
    exec(compile(_OPT_WP.read_text(), str(_OPT_WP), 'exec'), ns)
    wps = ns['WAYPOINTS']
    return np.array([[x, y] for x, y, _ in wps])


def ellipse_patch(b: GaussianBelief2D, n_std: float = 2.0, **kw) -> Ellipse:
    vals, vecs = np.linalg.eigh(b.cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    angle = float(np.degrees(np.arctan2(*vecs[:, 0][::-1])))
    w, h = 2.0 * n_std * np.sqrt(np.maximum(vals, 0.0))
    return Ellipse(xy=(b.mean[0], b.mean[1]), width=w, height=h, angle=angle, **kw)


def add_obstacles(ax):
    labels = [f'$\\mathcal{{O}}_{i}$' for i in range(1, len(OBSTACLES) + 1)]
    for (x0, x1, y0, y1), lbl in zip(OBSTACLES, labels):
        ax.add_patch(
            Rectangle(
                (x0, y0), x1 - x0, y1 - y0, facecolor='#d62728', alpha=0.30, edgecolor='#8b0000', lw=1.2, zorder=2
            )
        )
        ax.text(
            (x0 + x1) / 2,
            (y0 + y1) / 2,
            lbl,
            ha='center',
            va='center',
            fontsize=7,
            color='#8b0000',
            fontweight='bold',
            zorder=3,
        )


def ieee_style():
    plt.rcParams.update(
        {
            'font.family': 'serif',
            'font.size': 8,
            'axes.labelsize': 8,
            'axes.titlesize': 8,
            'xtick.labelsize': 7,
            'ytick.labelsize': 7,
            'legend.fontsize': 7,
            'figure.dpi': 150,
            'lines.linewidth': 1.2,
        }
    )


# =============================================================================
# Fig 1: Planned paths + 2-sigma ellipses
# =============================================================================

def fig1_paths():
    ieee_style()
    orig = sine_waypoints()
    opt = opt_waypoints()
    b_orig = propagate(orig)
    b_opt = propagate(opt)

    rho_orig = _eval_rho(b_orig)
    rho_opt = _eval_rho(b_opt)

    fig, axes = plt.subplots(1, 2, figsize=(6.5, 3.5), sharey=True)
    configs = [
        ('(a) Deterministic path', orig, b_orig, rho_orig, '#1f77b4', False),
        ('(b) pDSTL-optimised path', opt, b_opt, rho_opt, '#2ca02c', True),
    ]
    for ax, (title, wps, beliefs, rho, col, show_cov) in zip(axes, configs):
        add_obstacles(ax)
        if show_cov:
            for b in beliefs:
                ax.add_patch(ellipse_patch(b, facecolor=col, alpha=0.12, edgecolor=col, lw=0.7, zorder=3))
        ax.plot(wps[:, 0], wps[:, 1], 'o-', color=col, lw=1.5, ms=4, zorder=4)
        ax.plot(*START, '^', color='#2ca02c', ms=8, zorder=5, label='Start')
        ax.plot(*GOAL, '*', color='#d62728', ms=10, zorder=5, label='Goal')
        sat = 'satisfied' if rho >= ALPHA else 'not satisfied'
        ax.set_title(f'{title}\n$\\rho={rho:.4f}$, $\\alpha={ALPHA}$ ({sat})', pad=4)
        ax.set_xlabel('$x$ (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.25, lw=0.5)
        ax.legend(loc='lower right')
    axes[0].set_ylabel('$y$ (m)')
    fig.suptitle('Path optimisation with 2$\\sigma$ covariance ellipses', y=1.01)
    fig.tight_layout()
    out = _OUT / 'fig1_paths.pdf'
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved {out}')
    plt.close(fig)


def _eval_rho(beliefs):
    preds = [OutsideObstacle(*obs) for obs in OBSTACLES]
    per_step = []
    for b in beliefs:
        p = (1.0, 1.0)
        for pred in preds:
            p = stl_and(p, pred(b))
        per_step.append(p[0])
    return float(min(per_step))


# =============================================================================
# Reporting tables
# =============================================================================

def print_results_table(run_data: dict[tuple[int, str], list[list[dict]]]) -> None:
    """Print per-condition, per-speed statistics across all runs."""
    w = 118
    print('\n' + '=' * w)
    print('EXPERIMENT RESULTS — mean ± std across 20 runs per cell')
    print('=' * w)

    orig = sine_waypoints()
    opt  = opt_waypoints()

    col = (
        f"{'Fan':>4}  {'Runs':>4}  {'Samples':>7}  "
        f"{'Safe% mean':>10}  {'Safe% std':>9}  {'Viols':>5}  "
        f"{'MinClr mean':>11}  {'MinClr std':>10}  "
        f"{'Dev mean':>8}  {'Dev std':>7}"
    )

    for path_label, path_key, planned in [('DETERMINISTIC', 'det', orig), ('pDSTL-OPTIMISED', 'pdstl', opt)]:
        print(f'\n  {path_label}')
        print('  ' + '-' * (w - 2))
        print('  ' + col)
        print('  ' + '-' * (w - 2))

        for spd in FAN_SPEEDS:
            runs = run_data.get((spd, path_key))
            if not runs:
                print(f"  {spd:>4}  {'—':>4}")
                continue
            s = _run_stats(runs, planned)
            flag = ' ◄ VIOLATIONS' if s['viols_total'] > 0 else ''
            print(
                f"  {spd:>4}  {s['n_runs']:>4}  {s['n_samples']:>7}  "
                f"{s['sr_mean']:>10.1f}  {s['sr_std']:>9.1f}  {s['viols_total']:>5}  "
                f"{s['clr_mean']:>11.1f}  {s['clr_std']:>10.1f}  "
                f"{s['dev_mean']:>8.1f}  {s['dev_std']:>7.1f}{flag}"
            )

    print('\n' + '=' * w + '\n')


def print_latex_table(run_data: dict[tuple[int, str], list[list[dict]]]) -> None:
    """
    Print a booktabs LaTeX table:
      Fan speed | Det (Safe%, MinClr, Viols) | pDSTL (Safe%, MinClr, Viols)

    Values shown as mean ± std across runs.
    """
    orig = sine_waypoints()
    opt  = opt_waypoints()

    stats: dict = {}
    for spd in FAN_SPEEDS:
        for key, planned in [('det', orig), ('pdstl', opt)]:
            runs = run_data.get((spd, key))
            stats[(spd, key)] = _run_stats(runs, planned) if runs else None

    def _cell(s, metric_mean, metric_std) -> str:
        if s is None:
            return r'\textemdash'
        return f"${s[metric_mean]:.1f} \\pm {s[metric_std]:.1f}$"

    def _viols(s) -> str:
        if s is None:
            return r'\textemdash'
        return str(s['viols_total'])

    def _bold(val: str, better: bool) -> str:
        return f'\\textbf{{{val}}}' if better else val

    lines = [
        r'\begin{table}[t]',
        r'  \centering',
        r'  \caption{Safety rate (\%), minimum obstacle clearance (cm),',
        r'           and total constraint violations across 20 runs per cell',
        r'           for the deterministic and pDSTL-optimised paths.',
        r'           Values shown as mean\,$\pm$\,std across runs.}',
        r'  \label{tab:results}',
        r'  \setlength{\tabcolsep}{4pt}',
        r'  \begin{tabular}{c rr r rr r}',
        r'    \toprule',
        r'    & \multicolumn{3}{c}{Deterministic}',
        r'    & \multicolumn{3}{c}{pDSTL (ours)} \\',
        r'    \cmidrule(lr){2-4}\cmidrule(lr){5-7}',
        r'    \multirow{2}{*}{Fan}',
        r'    & Safe\,(\%)  & Min.\,clr.\,(cm)  & Viols.',
        r'    & Safe\,(\%)  & Min.\,clr.\,(cm)  & Viols. \\',
        r'    \midrule',
    ]

    for spd in FAN_SPEEDS:
        d = stats[(spd, 'det')]
        p = stats[(spd, 'pdstl')]
        lbl = f'{spd}\\,(off)' if spd == 0 else str(spd)

        sr_d  = _cell(d, 'sr_mean',  'sr_std')
        sr_p  = _cell(p, 'sr_mean',  'sr_std')
        cl_d  = _cell(d, 'clr_mean', 'clr_std')
        cl_p  = _cell(p, 'clr_mean', 'clr_std')
        vd    = _viols(d)
        vp    = _viols(p)

        if d and p:
            better_sr  = p['sr_mean']  >= d['sr_mean']
            better_clr = p['clr_mean'] >= d['clr_mean']
            better_vio = p['viols_total'] <= d['viols_total']
            sr_p  = _bold(sr_p,  better_sr)
            cl_p  = _bold(cl_p,  better_clr)
            vp    = _bold(vp,    better_vio)
            sr_d  = _bold(sr_d,  not better_sr)
            cl_d  = _bold(cl_d,  not better_clr)
            vd    = _bold(vd,    not better_vio)

        lines.append(f'    {lbl} & {sr_d} & {cl_d} & {vd} & {sr_p} & {cl_p} & {vp} \\\\')

    lines += [
        r'    \bottomrule',
        r'  \end{tabular}',
        r'\end{table}',
    ]

    print('\n% ── LaTeX table ─────────────────────────────────────────────────')
    print('\n'.join(lines))
    print('% ────────────────────────────────────────────────────────────────\n')


# =============================================================================
# Fig 2: trajectories at min/max wind + safety rate bar chart
# =============================================================================

def _mean_path(rows: list[dict], bin_width: float = 0.3):
    """Time-binned mean trajectory from actual Lighthouse samples."""
    if not rows:
        return [], []
    rows_s = sorted(rows, key=lambda r: r['t'])
    t_max = rows_s[-1]['t']
    bins = np.arange(0, t_max + bin_width, bin_width)
    xs, ys = [], []
    for t0, t1 in zip(bins[:-1], bins[1:]):
        bucket = [r for r in rows_s if t0 <= r['t'] < t1]
        if bucket:
            xs.append(sum(r['x'] for r in bucket) / len(bucket))
            ys.append(sum(r['y'] for r in bucket) / len(bucket))
    return xs, ys


def fig2_experiment(flat_data: dict[tuple[int, str], list[dict]]):
    """
    Args:
        flat_data: {(speed, path_type): [all rows pooled across all runs]}
    """
    ieee_style()

    orig = sine_waypoints()
    opt = opt_waypoints()

    # Safety rate per cell (over all pooled rows)
    safety: dict[tuple[int, str], float] = {}
    for (spd, pt), rows in flat_data.items():
        n = len(rows)
        viol = sum(1 for r in rows if not r['safe'])
        safety[(spd, pt)] = (n - viol) / n * 100 if n else 0.0

    fig = plt.figure(figsize=(6.5, 4.5))
    gs = fig.add_gridspec(2, 2, hspace=0.45, wspace=0.35)
    ax_l0 = fig.add_subplot(gs[0, 0])  # trajectories fan=0
    ax_ln = fig.add_subplot(gs[0, 1])  # trajectories fan=max
    ax_bar = fig.add_subplot(gs[1, :])  # bar chart

    col_det = '#1f77b4'
    col_pdstl = '#2ca02c'
    col_viol = '#d62728'

    def _traj_panel(ax, spd, title):
        add_obstacles(ax)
        for pt, col, lbl, cmd in [
            ('det', col_det, 'Det.', orig),
            ('pdstl', col_pdstl, 'pDSTL', opt),
        ]:
            rows = flat_data.get((spd, pt), [])
            safe_x = [r['x'] for r in rows if r['safe']]
            safe_y = [r['y'] for r in rows if r['safe']]
            viol_x = [r['x'] for r in rows if not r['safe']]
            viol_y = [r['y'] for r in rows if not r['safe']]
            ax.plot(cmd[:, 0], cmd[:, 1], '--', color=col, lw=0.8, alpha=0.5, zorder=3)
            ax.scatter(safe_x, safe_y, s=4, color=col, alpha=0.25, zorder=4)
            if viol_x:
                ax.scatter(viol_x, viol_y, s=18, color=col_viol, marker='x', lw=1.0, zorder=5,
                           label=f'{lbl} violation')
            mx, my = _mean_path(rows)
            if mx:
                ax.plot(mx, my, '-', color=col, lw=1.8, zorder=6, label=f'{lbl} mean')
        ax.plot(*START, '^', color='#2ca02c', ms=6, zorder=6)
        ax.plot(*GOAL, '*', color='#d62728', ms=8, zorder=6)
        ax.set_title(title, pad=3)
        ax.set_xlabel('$x$ (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.2, lw=0.4)
        ax.legend(loc='lower right', markerscale=1.5, handlelength=1)

    _traj_panel(ax_l0, FAN_SPEEDS[0],  f'(a) Fan speed {FAN_SPEEDS[0]} (off)')
    _traj_panel(ax_ln, FAN_SPEEDS[-1], f'(b) Fan speed {FAN_SPEEDS[-1]} (max)')
    ax_l0.set_ylabel('$y$ (m)')

    # Bar chart — all fan speeds
    sr_det   = [safety.get((spd, 'det'),   0.0) for spd in FAN_SPEEDS]
    sr_pdstl = [safety.get((spd, 'pdstl'), 0.0) for spd in FAN_SPEEDS]
    x = np.arange(N_LEVELS)
    w = 0.35
    bars_d = ax_bar.bar(x - w / 2, sr_det,   w, label='Deterministic', color=col_det,   alpha=0.85, edgecolor='k', lw=0.5)
    bars_p = ax_bar.bar(x + w / 2, sr_pdstl, w, label='pDSTL',         color=col_pdstl, alpha=0.85, edgecolor='k', lw=0.5)
    ax_bar.axhline(ALPHA * 100, color='#d62728', lw=1.2, ls='--', label=f'$\\alpha = {ALPHA}$')
    for bar in list(bars_d) + list(bars_p):
        h = bar.get_height()
        ax_bar.text(bar.get_x() + bar.get_width() / 2, h + 0.2, f'{h:.0f}',
                    ha='center', va='bottom', fontsize=5.5)
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels([f'{s}\n(off)' if s == 0 else str(s) for s in FAN_SPEEDS])
    ax_bar.set_xlabel('Fan speed')
    ax_bar.set_ylabel('Safety rate (%)')
    _ymin = max(0, min(sr_det + sr_pdstl) - 5)
    ax_bar.set_ylim(_ymin, 102.5)
    ax_bar.set_title('(c) Empirical safety rate vs fan speed', pad=3)
    ax_bar.legend(loc='lower left')
    ax_bar.grid(True, axis='y', alpha=0.25, lw=0.5)

    out = _OUT / 'fig2_experiment.pdf'
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved {out}')
    plt.close(fig)


# =============================================================================
# Fig 3: Min obstacle clearance vs fan speed (mean ± std across runs)
# =============================================================================

def fig3_clearance(run_data: dict[tuple[int, str], list[list[dict]]]):
    """
    Args:
        run_data: {(speed, path_type): [[rows_run1], [rows_run2], ...]}
    """
    ieee_style()

    def _per_run_min_clr(runs):
        """Per-run minimum clearance (cm) list."""
        clrs = []
        for rows in runs:
            if rows:
                clrs.append(min(_min_obs_clearance(r['x'], r['y']) for r in rows))
        return np.array(clrs) if clrs else np.array([0.0])

    col_det   = '#1f77b4'
    col_pdstl = '#2ca02c'

    clr_det_mean, clr_det_std     = [], []
    clr_pdstl_mean, clr_pdstl_std = [], []

    for spd in FAN_SPEEDS:
        for pt, mean_list, std_list in [('det', clr_det_mean, clr_det_std),
                                         ('pdstl', clr_pdstl_mean, clr_pdstl_std)]:
            runs = run_data.get((spd, pt), [])
            arr = _per_run_min_clr(runs)
            mean_list.append(arr.mean())
            std_list.append(arr.std(ddof=1) if len(arr) > 1 else 0.0)

    fig, ax = plt.subplots(figsize=(4.0, 2.8))
    ax.errorbar(FAN_SPEEDS, clr_det_mean,   yerr=clr_det_std,   fmt='o-', color=col_det,
                capsize=3, label='Deterministic', ms=5)
    ax.errorbar(FAN_SPEEDS, clr_pdstl_mean, yerr=clr_pdstl_std, fmt='s-', color=col_pdstl,
                capsize=3, label='pDSTL', ms=5)
    ax.axhline(0, color='#d62728', lw=1.0, ls='--', label='Obstacle boundary')
    ax.fill_between(FAN_SPEEDS, 0, min(*clr_det_mean, *clr_pdstl_mean) - 0.5,
                    color='#d62728', alpha=0.10)
    ax.set_xlabel('Fan speed')
    ax.set_ylabel('Min. obstacle clearance (cm)')
    ax.set_title('Minimum obstacle clearance vs fan speed\n(mean ± std across runs)', pad=3)
    ax.set_xticks(FAN_SPEEDS)
    ax.set_xticklabels([f'{s}\n(off)' if s == 0 else str(s) for s in FAN_SPEEDS])
    ax.legend()
    ax.grid(True, alpha=0.25, lw=0.5)
    fig.tight_layout()
    out = _OUT / 'fig3_clearance.pdf'
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved {out}')
    plt.close(fig)


# =============================================================================
# Fig 4: All fan speeds overlaid — one panel per configuration
# =============================================================================

def fig4_all_speeds(flat_data: dict[tuple[int, str], list[dict]]):
    """
    Args:
        flat_data: {(speed, path_type): [all rows pooled across all runs]}
    """
    ieee_style()

    cmap = plt.get_cmap('plasma')
    n = max(len(FAN_SPEEDS), 1)
    speed_colour = {spd: cmap(i / (n - 1) if n > 1 else 0.0) for i, spd in enumerate(FAN_SPEEDS)}

    fig, axes = plt.subplots(1, 2, figsize=(6.5, 3.8), sharey=True)
    titles = ['(a) Deterministic — all fan speeds', '(b) pDSTL — all fan speeds']
    path_keys = ['det', 'pdstl']

    for ax, pt, title in zip(axes, path_keys, titles):
        add_obstacles(ax)
        for spd in FAN_SPEEDS:
            rows = flat_data.get((spd, pt), [])
            mx, my = _mean_path(rows)
            if mx:
                lbl = f'fan {spd}' + (' (off)' if spd == 0 else '')
                ax.plot(mx, my, '-', color=speed_colour[spd], lw=1.4, label=lbl, zorder=4)
        ax.plot(*START, '^', color='#2ca02c', ms=7, zorder=6)
        ax.plot(*GOAL, '*', color='#d62728', ms=9, zorder=6)
        ax.set_title(title, pad=3)
        ax.set_xlabel('$x$ (m)')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.25, lw=0.5)
        ax.legend(loc='lower right', fontsize=6, handlelength=1.2)

    axes[0].set_ylabel('$y$ (m)')

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=FAN_SPEEDS[0], vmax=FAN_SPEEDS[-1]))
    sm.set_array([])
    fig.colorbar(sm, ax=axes, orientation='vertical', label='Fan speed', shrink=0.85, pad=0.02)

    fig.suptitle('Actual mean trajectories at all fan speeds', y=1.01)
    fig.tight_layout()
    out = _OUT / 'fig4_all_speeds.pdf'
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved {out}')
    plt.close(fig)


# =============================================================================
if __name__ == '__main__':
    # Load per-run data for statistical analysis
    RUN_DATA = load_all_runs(FAN_MAP)

    # Flat (pooled) data for trajectory figures
    FLAT_DATA: dict[tuple[int, str], list[dict]] = {
        k: [r for run in runs for r in run]
        for k, runs in RUN_DATA.items()
    }

    if not RUN_DATA:
        print('[plot_results] No log files found in', _LOGS)
        print('  Expected filename format: <condition>_fan<XX>_run<NN>_<ts>_actual.csv')
    else:
        print(f'[plot_results] Found {sum(len(v) for v in RUN_DATA.values())} runs '
              f'across {len(RUN_DATA)} cells.')

    fig1_paths()
    print_results_table(RUN_DATA)
    print_latex_table(RUN_DATA)
    fig2_experiment(FLAT_DATA)
    fig3_clearance(RUN_DATA)
    fig4_all_speeds(FLAT_DATA)
    print(f'\nAll figures saved to {_OUT}/')

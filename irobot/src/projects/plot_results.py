"""
Publication figures for the pDSTL Crazyflie experiment.

Generates three figures:
  fig1_paths.pdf      — planned paths + 2-sigma covariance ellipses (before/after)
  fig2_experiment.pdf — 3-panel: actual trajectories at max wind + safety rate bar chart
  fig3_clearance.pdf  — minimum obstacle clearance vs fan level

Run from the repo root:
    python irobot/src/projects/plot_results.py
"""

from __future__ import annotations

import csv
import pathlib
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

# ── Constants (must match optimise_cf_path.py) ────────────────────────────────
START = np.array([0.0, -1.5])
GOAL = np.array([0.489, 0.65])
N_WP = 10
OBSTACLES = [
    (-0.25, 0.25, -1.05, -0.85),  # OBS-1
    (-0.45,  0.05, 0.10, 0.35),  # OBS-2
    ( 0.20,  0.55, -0.25, 0.10),  # OBS-3
]
SIGMA0 = np.eye(2) * (0.010**2)
SIGMA_TOTAL_STEP = float(np.sqrt(0.010**2 + 0.020**2))
DT = 0.1
ALPHA = 0.90
FWD_CUTOFF = 7.0


def _build_fan_map() -> tuple[dict[str, tuple[int, str]], list[int]]:
    """
    Auto-scan _LOGS for actual CSVs with the fan<XX> tag in the filename.
    Returns (FAN_MAP, sorted unique fan speeds).
    Filename format: <condition>_fan<XX>_<ts>_actual.csv
    """
    import re

    fan_map: dict[str, tuple[int, str]] = {}
    _DET = {'deterministic_nominal', 'deterministic_wind'}
    _PDSTL = {'pdstl_nominal', 'pdstl_wind'}
    pattern = re.compile(r'^(?P<cond>\w+)_fan(?P<speed>\d+)_\d{8}T\d{6}_actual\.csv$')
    for f in sorted(_LOGS.glob('*_actual.csv')):
        m = pattern.match(f.name)
        if not m:
            continue
        cond = m.group('cond')
        speed = int(m.group('speed'))
        path_type = 'det' if cond in _DET else 'pdstl' if cond in _PDSTL else None
        if path_type is None:
            continue
        fan_map[f.name] = (speed, path_type)
    speeds = sorted({v[0] for v in fan_map.values()})
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
# Fig 2: 3-panel experiment figure
#   (a) Actual trajectories at fan level 3 (det vs pdstl)
#   (b) Actual trajectories at fan level 0 (det vs pdstl)
#   (c) Safety rate vs fan level bar chart
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


def _min_obs_clearance(x: float, y: float) -> float:
    """Minimum Euclidean clearance from point (x,y) to any obstacle boundary (cm)."""
    dists = []
    for x0, x1, y0, y1 in OBSTACLES:
        dx = max(x0 - x, 0.0, x - x1)
        dy = max(y0 - y, 0.0, y - y1)
        dists.append(float(np.hypot(dx, dy)) * 100.0)
    return min(dists)


def print_results_table(data: dict) -> None:
    """Print a detailed deviation report to the terminal."""
    w = 108
    print('\n' + '=' * w)
    print('EXPERIMENT RESULTS — deviations from planned path & obstacle safety')
    print('=' * w)

    orig = sine_waypoints()
    opt  = opt_waypoints()

    col = f"{'Fan':>4}  {'Samples':>7}  {'Safe%':>6}  {'Viols':>5}  "
    col += f"{'MinClr(cm)':>10}  {'MeanDev(cm)':>11}  {'MaxDev(cm)':>10}  {'MeanX(m)':>8}  {'MeanY(m)':>8}"

    for path_label, path_key, planned in [('DETERMINISTIC', 'det', orig), ('pDSTL-OPTIMISED', 'pdstl', opt)]:
        print(f'\n  {path_label}')
        print('  ' + '-' * (w - 2))
        print('  ' + col)
        print('  ' + '-' * (w - 2))

        planned_xy = planned  # (N_WP, 2)

        for spd in FAN_SPEEDS:
            rows = data.get((spd, path_key), [])
            if not rows:
                print(f"  {spd:>4}  {'—':>7}")
                continue

            n      = len(rows)
            viols  = sum(1 for r in rows if not r['safe'])
            safe_p = (n - viols) / n * 100.0

            # Min obstacle clearance over the flight
            min_clr = min(_min_obs_clearance(r['x'], r['y']) for r in rows)

            # Mean deviation from nearest planned waypoint
            devs = []
            for r in rows:
                dists_to_wps = [float(np.hypot(r['x'] - px, r['y'] - py)) for px, py in planned_xy]
                devs.append(min(dists_to_wps) * 100.0)
            mean_dev = sum(devs) / len(devs)
            max_dev  = max(devs)

            mean_x = sum(r['x'] for r in rows) / n
            mean_y = sum(r['y'] for r in rows) / n

            flag = ' ◄ VIOLATION' if viols > 0 else ''
            print(
                f'  {spd:>4}  {n:>7}  {safe_p:>6.1f}  {viols:>5}  '
                f'{min_clr:>10.1f}  {mean_dev:>11.1f}  {max_dev:>10.1f}  '
                f'{mean_x:>8.3f}  {mean_y:>8.3f}{flag}'
            )

    print('\n' + '=' * w + '\n')


def fig2_experiment():
    ieee_style()

    # Load all actual logs
    data: dict[tuple[int, str], list[dict]] = {}
    for fname, (level, path) in FAN_MAP.items():
        rows = load_csv(_LOGS / fname)
        data[(level, path)] = [r for r in rows if r['t'] <= FWD_CUTOFF]

    orig = sine_waypoints()
    opt = opt_waypoints()

    # Safety rate table
    safety: dict[tuple[int, str], float] = {}
    for (level, path), rows in data.items():
        n = len(rows)
        viol = sum(1 for r in rows if not r['safe'])
        safety[(level, path)] = (n - viol) / n * 100

    fig = plt.figure(figsize=(6.5, 4.5))
    gs = fig.add_gridspec(2, 2, hspace=0.45, wspace=0.35)
    ax_l0 = fig.add_subplot(gs[0, 0])  # trajectories fan=0
    ax_l3 = fig.add_subplot(gs[0, 1])  # trajectories fan=3
    ax_bar = fig.add_subplot(gs[1, :])  # bar chart

    col_det = '#1f77b4'
    col_pdstl = '#2ca02c'
    col_viol = '#d62728'

    def _traj_panel(ax, level, title):
        add_obstacles(ax)
        for path, col, lbl, cmd in [
            ('det', col_det, 'Det.', orig),
            ('pdstl', col_pdstl, 'pDSTL', opt),
        ]:
            rows = data[(level, path)]
            xs = [r['x'] for r in rows]
            ys = [r['y'] for r in rows]
            # colour unsafe points red
            safe_x = [r['x'] for r in rows if r['safe']]
            safe_y = [r['y'] for r in rows if r['safe']]
            viol_x = [r['x'] for r in rows if not r['safe']]
            viol_y = [r['y'] for r in rows if not r['safe']]
            ax.plot(cmd[:, 0], cmd[:, 1], '--', color=col, lw=0.8, alpha=0.5, zorder=3)
            ax.scatter(safe_x, safe_y, s=4, color=col, alpha=0.35, zorder=4)
            if viol_x:
                ax.scatter(viol_x, viol_y, s=18, color=col_viol, marker='x', lw=1.0, zorder=5, label=f'{lbl} violation')
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

    _traj_panel(ax_l0, FAN_SPEEDS[0], f'(a) Fan speed {FAN_SPEEDS[0]} (off)')
    _traj_panel(ax_l3, FAN_SPEEDS[-1], f'(b) Fan speed {FAN_SPEEDS[-1]} (max)')
    ax_l0.set_ylabel('$y$ (m)')

    # Bar chart — all fan speeds
    levels = FAN_SPEEDS
    sr_det = [safety[(l, 'det')] for l in levels]
    sr_pdstl = [safety[(l, 'pdstl')] for l in levels]
    x = np.arange(N_LEVELS)
    w = 0.35
    bars_d = ax_bar.bar(x - w / 2, sr_det, w, label='Deterministic', color=col_det, alpha=0.85, edgecolor='k', lw=0.5)
    bars_p = ax_bar.bar(x + w / 2, sr_pdstl, w, label='pDSTL', color=col_pdstl, alpha=0.85, edgecolor='k', lw=0.5)
    ax_bar.axhline(ALPHA * 100, color='#d62728', lw=1.2, ls='--', label=f'$\\alpha = {ALPHA}$')
    for bar in list(bars_d) + list(bars_p):
        h = bar.get_height()
        ax_bar.text(bar.get_x() + bar.get_width() / 2, h + 0.2, f'{h:.0f}', ha='center', va='bottom', fontsize=5.5)
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels([f'{s}\n(off)' if s == 0 else str(s) for s in FAN_SPEEDS])
    ax_bar.set_xlabel('Fan speed')
    ax_bar.set_ylabel('Safety rate (%)')
    ax_bar.set_ylim(88, 102.5)
    ax_bar.set_title('(c) Empirical safety rate vs fan speed', pad=3)
    ax_bar.legend(loc='lower left')
    ax_bar.grid(True, axis='y', alpha=0.25, lw=0.5)

    out = _OUT / 'fig2_experiment.pdf'
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved {out}')
    plt.close(fig)


# =============================================================================
# Fig 3: Min obstacle clearance vs fan level
# =============================================================================


def fig3_clearance():
    ieee_style()

    def min_obs_dist(px, py):
        dists = []
        for x0, x1, y0, y1 in OBSTACLES:
            dx = max(x0 - px, 0, px - x1)
            dy = max(y0 - py, 0, py - y1)
            dists.append(float(np.hypot(dx, dy)))
        return min(dists)

    min_d: dict[tuple[int, str], float] = {}
    for fname, (level, path) in FAN_MAP.items():
        rows = [r for r in load_csv(_LOGS / fname) if r['t'] <= FWD_CUTOFF]
        min_d[(level, path)] = min(min_obs_dist(r['x'], r['y']) * 100 for r in rows)

    col_det = '#1f77b4'
    col_pdstl = '#2ca02c'

    fig, ax = plt.subplots(figsize=(4.0, 2.8))
    ax.plot(FAN_SPEEDS, [min_d[(l, 'det')] for l in FAN_SPEEDS], 'o-', color=col_det, label='Deterministic', ms=5)
    ax.plot(FAN_SPEEDS, [min_d[(l, 'pdstl')] for l in FAN_SPEEDS], 's-', color=col_pdstl, label='pDSTL', ms=5)
    ax.axhline(0, color='#d62728', lw=1.0, ls='--', label='Obstacle boundary')
    ax.fill_between(FAN_SPEEDS, 0, min(min_d.values()) - 0.5, color='#d62728', alpha=0.10)
    ax.set_xlabel('Fan speed')
    ax.set_ylabel('Min. obstacle clearance (cm)')
    ax.set_title('Minimum obstacle clearance vs fan speed', pad=3)
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
# Fig 4: All fan speeds overlaid — one panel per configuration (det / pDSTL)
# =============================================================================


def fig4_all_speeds() -> None:
    ieee_style()

    # Load all actual logs
    data: dict[tuple[int, str], list[dict]] = {}
    for fname, (spd, pt) in FAN_MAP.items():
        rows = load_csv(_LOGS / fname)
        data[(spd, pt)] = [r for r in rows if r['t'] <= FWD_CUTOFF]

    cmap = plt.get_cmap('plasma')
    n = max(len(FAN_SPEEDS), 1)
    speed_colour = {spd: cmap(i / (n - 1) if n > 1 else 0.0) for i, spd in enumerate(FAN_SPEEDS)}

    fig, axes = plt.subplots(1, 2, figsize=(6.5, 3.8), sharey=True)
    titles = ['(a) Deterministic — all fan speeds', '(b) pDSTL — all fan speeds']
    path_keys = ['det', 'pdstl']

    for ax, pt, title in zip(axes, path_keys, titles):
        add_obstacles(ax)
        for spd in FAN_SPEEDS:
            rows = data.get((spd, pt), [])
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

    # Shared colorbar for fan speed
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
    fig1_paths()
    # Load data once for table + fig2
    _data: dict = {}
    for _fname, (_spd, _pt) in FAN_MAP.items():
        _rows = load_csv(_LOGS / _fname)
        _data[(_spd, _pt)] = [r for r in _rows if r['t'] <= FWD_CUTOFF]
    print_results_table(_data)
    fig2_experiment()
    fig3_clearance()
    fig4_all_speeds()
    print(f'\nAll figures saved to {_OUT}/')

"""
Publication figures for the pDSTL Crazyflie experiment.

Generates four figures:
  fig1_paths.pdf      — planned paths + 2-sigma covariance ellipses (before/after)
  fig2_experiment.pdf — mean paths at all fan speeds per condition (det | pDSTL), scatter background
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

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
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
    (-0.165, 0.165, -1.144, -0.941),  # OBS-1 (red)
    (-0.432, -0.102, -0.179, 0.049),  # OBS-2 (blue)
    (0.114, 0.343, -0.611, -0.421),  # OBS-3 (green)
]
SIGMA0 = np.eye(2) * (0.010**2)
SIGMA_TOTAL_STEP = float(np.sqrt(0.010**2 + 0.020**2))
DT = 0.1
ALPHA = 0.90
FWD_CUTOFF = 11.0  # s — covers full forward flight (logger starts ~3 s before first waypoint)
GOAL_TOL = 0.15  # metres — within 15 cm of GOAL counts as reached
GOAL_WINDOW = 2.0  # seconds — look at samples in the last 2 s of each run
# Common arena limits applied to all trajectory panels so both panels are identical in size.
# x range 2.0 m, y range 2.55 m → aspect ratio 1.275 (panels stay compact in a 2-col figure).
_XLIM = (-1.0, 1.0)
_YLIM = (-1.70, 0.85)


# =============================================================================
# Data loading — multi-run aware
# =============================================================================


def _build_fan_map() -> tuple[dict[tuple[int, str], list[pathlib.Path]], list[int], set[pathlib.Path]]:
    """
    Auto-scan _LOGS for actual CSVs produced by the new naming scheme:
        <condition>_fan<XX>_run<NN>_<ts>_actual.csv          ← completed run
        <condition>_fan<XX>_run<NN>_CRASH_<ts>_actual.csv   ← crashed run

    Both are included. Crash paths are returned separately so load_all_runs
    can mark their rows, making them automatically count as violations.

    Returns:
        fan_map     : {(fan_speed, path_type): [list of matching Path objects]}
        speeds      : sorted list of unique fan speeds found
        crash_paths : set of Path objects that are crash files
    """
    pattern_ok = re.compile(
        r'^(?P<cond>deterministic|pdstl)_fan(?P<speed>\d+)_run(?P<run>\d+)'
        r'_(?P<ts>\d{8}T\d{6})_actual\.csv$'
    )
    pattern_crash = re.compile(
        r'^(?P<cond>deterministic|pdstl)_fan(?P<speed>\d+)_run(?P<run>\d+)'
        r'_CRASH_(?P<ts>\d{8}T\d{6})_actual\.csv$'
    )
    fan_map: dict[tuple[int, str], list[pathlib.Path]] = {}
    crash_paths: set[pathlib.Path] = set()

    for f in sorted(_LOGS.glob('*_actual.csv')):
        m = pattern_ok.match(f.name) or pattern_crash.match(f.name)
        if not m:
            continue  # old-format or unrelated files
        cond = m.group('cond')
        speed = int(m.group('speed'))
        path_type = 'det' if cond == 'deterministic' else 'pdstl'
        key = (speed, path_type)
        fan_map.setdefault(key, []).append(f)
        if pattern_crash.match(f.name):
            crash_paths.add(f)

    speeds = sorted({k[0] for k in fan_map})
    return fan_map, speeds, crash_paths


FAN_MAP, FAN_SPEEDS, _CRASH_PATHS = _build_fan_map()
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
    fan_map: dict[tuple[int, str], list[pathlib.Path]],
    crash_paths: set[pathlib.Path] | None = None,
) -> dict[tuple[int, str], list[list[dict]]]:
    """
    Load every CSV for each (speed, path_type) cell.

    Crash files (identified via crash_paths) have every row marked with
    r['_crash'] = True so that _run_stats counts them as violations regardless
    of whether the partial Lighthouse samples happened to be in a safe zone.

    Returns:
        {(speed, path_type): [[rows_run1], [rows_run2], ...]}
        Each inner list contains only rows with t <= FWD_CUTOFF.
    """
    crash_paths = crash_paths or set()
    result: dict[tuple[int, str], list[list[dict]]] = {}
    for key, paths in fan_map.items():
        runs = []
        for p in paths:
            rows = [r for r in load_csv(p) if r['t'] <= FWD_CUTOFF]
            if p in crash_paths:
                for r in rows:
                    r['_crash'] = True
            # Store per-run goal from last row of commanded CSV
            cmd_p = p.with_name(p.name.replace('_actual.csv', '_commanded.csv'))
            if rows and cmd_p.exists():
                with cmd_p.open() as fh:
                    cmd_rows = list(csv.DictReader(fh))
                if cmd_rows:
                    rows[0]['_goal_x'] = float(cmd_rows[-1]['x'])
                    rows[0]['_goal_y'] = float(cmd_rows[-1]['y'])
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


def _reached_goal(rows: list[dict]) -> bool:
    """True if any sample in the last GOAL_WINDOW seconds is within GOAL_TOL of per-run goal."""
    if not rows:
        return False
    gx = rows[0].get('_goal_x', GOAL[0])
    gy = rows[0].get('_goal_y', GOAL[1])
    t_max = max(r['t'] for r in rows)
    late = [r for r in rows if r['t'] >= t_max - GOAL_WINDOW]
    return any(float(np.hypot(r['x'] - gx, r['y'] - gy)) <= GOAL_TOL for r in late)


def _run_stats(runs: list[list[dict]], planned_xy: np.ndarray) -> dict:
    """
    A run is VIOLATED if: obstacle entered, OR flagged crash, OR goal not reached.
    Violated runs are assigned 0 cm clearance.
    Clearance reported as mean ± std across all runs (0 for violated).
    """
    clr_all, dev_list = [], []
    n_total = 0
    n_violated = 0

    for rows in runs:
        is_crash = bool(rows[0].get('_crash', False)) if rows else False
        reached = _reached_goal(rows)
        violated = is_crash or any(not r['safe'] for r in rows) or not reached
        n_violated += 1 if violated else 0
        clr_all.append(0.0 if violated else min(_min_obs_clearance(r['x'], r['y']) for r in rows))
        devs = [min(float(np.hypot(r['x'] - px, r['y'] - py)) for px, py in planned_xy) * 100.0 for r in rows]
        dev_list.append(sum(devs) / len(devs))
        n_total += len(rows)

    arr_clr = np.array(clr_all)
    arr_dev = np.array(dev_list)
    n_runs = len(runs)

    _, _, cov_bins = _mean_path_with_cov(runs)
    pos_cov_trace_cm2 = float(np.mean([np.trace(c) for c in cov_bins])) * 1e4 if cov_bins else 0.0

    return {
        'n_runs': n_runs,
        'n_violated': n_violated,
        'success_pct': (n_runs - n_violated) / n_runs * 100.0 if n_runs > 0 else 0.0,
        'clr_mean': float(arr_clr.mean()),
        'clr_std': float(arr_clr.std(ddof=1)) if n_runs > 1 else 0.0,
        'dev_mean': float(arr_dev.mean()),
        'dev_std': float(arr_dev.std(ddof=1)) if n_runs > 1 else 0.0,
        'n_samples': n_total,
        'pos_cov_trace_cm2': pos_cov_trace_cm2,
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
    colors = ['#d62728', '#1f77b4', '#2ca02c']
    edge_colors = ['#8b0000', '#08519c', '#006d2c']
    labels = [f'$\\mathcal{{O}}_{i}$' for i in range(1, len(OBSTACLES) + 1)]
    for (x0, x1, y0, y1), lbl, fc, ec in zip(OBSTACLES, labels, colors, edge_colors):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=fc, alpha=0.30, edgecolor=ec, lw=1.2, zorder=2))
        ax.text(
            (x0 + x1) / 2,
            (y0 + y1) / 2,
            lbl,
            ha='center',
            va='center',
            fontsize=12,
            color=ec,
            fontweight='bold',
            zorder=3,
        )


def ieee_style():
    plt.rcParams.update(
        {
            'font.family': 'sans-serif',
            'font.size': 16,
            'axes.labelsize': 16,
            'axes.titlesize': 18,
            'xtick.labelsize': 14,
            'ytick.labelsize': 14,
            'legend.fontsize': 14,
            'figure.dpi': 150,
            'lines.linewidth': 2.0,
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

    fig, axes = plt.subplots(1, 2, figsize=(10.0, 5.5), sharey=True)
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
        ax.set_xlim(*_XLIM)
        ax.set_ylim(*_YLIM)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.25, lw=0.5)
        ax.legend(loc='lower right')
    axes[0].set_ylabel('$y$ (m)')

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
    w = 110
    print('\n' + '=' * w)
    print('EXPERIMENT RESULTS — success = safe + goal reached; clearance mean ± std (0 for failures)')
    print('=' * w)

    orig = sine_waypoints()
    opt = opt_waypoints()

    col = (
        f'{"Fan":>4}  {"Runs":>4}  {"Success%":>9}  {"Viols":>5}  '
        f'{"Clr mean":>8}  {"Clr std":>7}  '
        f'{"Dev mean":>8}  {"Dev std":>7}'
    )

    for path_label, path_key, planned in [('DETERMINISTIC', 'det', orig), ('pDSTL-OPTIMISED', 'pdstl', opt)]:
        print(f'\n  {path_label}')
        print('  ' + '-' * (w - 2))
        print('  ' + col)
        print('  ' + '-' * (w - 2))

        for spd in FAN_SPEEDS:
            runs = run_data.get((spd, path_key))
            if not runs:
                print(f'  {spd:>4}  {"—":>4}')
                continue
            s = _run_stats(runs, planned)
            flag = ' ◄ FAILURES' if s['n_violated'] > 0 else ''
            print(
                f'  {spd:>4}  {s["n_runs"]:>4}  {s["success_pct"]:>9.1f}  {s["n_violated"]:>5}  '
                f'{s["clr_mean"]:>8.1f}  {s["clr_std"]:>7.1f}  '
                f'{s["dev_mean"]:>8.1f}  {s["dev_std"]:>7.1f}{flag}'
            )

    print('\n' + '=' * w + '\n')


def print_latex_table(run_data: dict[tuple[int, str], list[list[dict]]]) -> None:
    """
    Print a booktabs LaTeX table:
      Fan speed | Det (Safe%, MinClr, Viols) | pDSTL (Safe%, MinClr, Viols)

    Safety: P(safe run) = (N - violations) / N * 100 % -- single number, no +/- std.
    Clearance: mean ± std over non-violated runs only.
    """
    orig = sine_waypoints()
    opt = opt_waypoints()

    stats: dict = {}
    for spd in FAN_SPEEDS:
        for key, planned in [('det', orig), ('pdstl', opt)]:
            runs = run_data.get((spd, key))
            stats[(spd, key)] = _run_stats(runs, planned) if runs else None

    def _pct(s, key) -> str:
        if s is None:
            return r'\textemdash'
        return f'${s[key]:.1f}$'

    def _viols(s) -> str:
        if s is None:
            return r'\textemdash'
        return str(s['n_violated'])

    def _bold(val: str, better: bool) -> str:
        return f'\\textbf{{{val}}}' if better else val

    lines = [
        r'\begin{table}[t]',
        r'  \centering',
        r'  \caption{Safety rate (\%), goal-reached rate (\%), minimum obstacle clearance (cm),',
        r'           and violation count across runs per cell,',
        r'           for the deterministic and pDSTL-optimised paths.',
        r'           Safety: $P(\text{safe run}) = (N-\text{violations})/N \times 100\,\%$.',
        r'           Clearance: median across all runs (violated/crashed runs assigned 0\,cm).}',
        r'  \label{tab:results}',
        r'  \setlength{\tabcolsep}{4pt}',
        r'  \begin{tabular}{c rrr rrr}',
        r'    \toprule',
        r'    & \multicolumn{3}{c}{Deterministic}',
        r'    & \multicolumn{3}{c}{pDSTL (ours)} \\',
        r'    \cmidrule(lr){2-4}\cmidrule(lr){5-7}',
        r'    Fan',
        r'    & Success\,(\%)  & Clr.\,(cm)  & Fails',
        r'    & Success\,(\%)  & Clr.\,(cm)  & Fails \\',
        r'    \midrule',
    ]

    for spd in FAN_SPEEDS:
        d = stats[(spd, 'det')]
        p = stats[(spd, 'pdstl')]
        lbl = f'{spd}\\,(off)' if spd == 0 else str(spd)

        sr_d = _pct(d, 'success_pct')
        sr_p = _pct(p, 'success_pct')
        cl_d = f'${d["clr_mean"]:.1f} \\pm {d["clr_std"]:.1f}$' if d else r'\textemdash'
        cl_p = f'${p["clr_mean"]:.1f} \\pm {p["clr_std"]:.1f}$' if p else r'\textemdash'
        vd = _viols(d)
        vp = _viols(p)

        if d and p:
            better_sr = p['success_pct'] >= d['success_pct']
            better_clr = p['clr_mean'] >= d['clr_mean']
            better_vio = p['n_violated'] <= d['n_violated']
            sr_p = _bold(sr_p, better_sr)
            cl_p = _bold(cl_p, better_clr)
            vp = _bold(vp, better_vio)
            sr_d = _bold(sr_d, not better_sr)
            cl_d = _bold(cl_d, not better_clr)
            vd = _bold(vd, not better_vio)

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


def _mean_path_with_cov(run_rows_list: list[list[dict]], bin_width: float = 0.3) -> tuple[list, list, list]:
    """
    Ensemble mean path and 2×2 sample covariance at each time bin.

    Each run is time-binned independently (equal weight per run).
    At each bin we have one (x, y) per run; np.cov gives the 2×2 empirical
    covariance across the 20 runs — directly analogous to the pDSTL model
    covariance in fig1, but computed from actual flight data.

    Returns:
        mean_x, mean_y — ensemble mean position at each time bin
        cov_list       — list of (2, 2) ndarrays; cov_list[i] is the
                         cross-run covariance matrix at bin i
    """
    # Exclude crash runs — their truncated paths would bias late time bins.
    run_rows_list = [rows for rows in run_rows_list if rows and not rows[0].get('_crash', False)]
    if not run_rows_list:
        return [], [], []
    t_maxes = [max(r['t'] for r in rows) for rows in run_rows_list if rows]
    if not t_maxes:
        return [], [], []
    t_max = min(t_maxes)
    bins = np.arange(0, t_max + bin_width, bin_width)
    n_bins = len(bins) - 1
    if n_bins < 1:
        return [], [], []

    per_x: list[list] = []
    per_y: list[list] = []
    for rows in run_rows_list:
        rx, ry = [], []
        for t0, t1 in zip(bins[:-1], bins[1:]):
            bucket = [r for r in rows if t0 <= r['t'] < t1]
            if bucket:
                rx.append(float(np.mean([r['x'] for r in bucket])))
                ry.append(float(np.mean([r['y'] for r in bucket])))
            else:
                rx.append(None)
                ry.append(None)
        per_x.append(rx)
        per_y.append(ry)

    mean_x, mean_y, cov_list = [], [], []
    for b in range(n_bins):
        xs_b = [per_x[i][b] for i in range(len(run_rows_list)) if per_x[i][b] is not None]
        ys_b = [per_y[i][b] for i in range(len(run_rows_list)) if per_y[i][b] is not None]
        if len(xs_b) < 2:
            continue
        mean_x.append(float(np.mean(xs_b)))
        mean_y.append(float(np.mean(ys_b)))
        cov_list.append(np.cov(xs_b, ys_b, ddof=1))  # (2, 2) ndarray

    return mean_x, mean_y, cov_list


def _cov_ellipse_patch(mx: float, my: float, cov: np.ndarray, n_std: float = 2.0, **kw) -> Ellipse:
    """Return a matplotlib Ellipse for the n-sigma empirical covariance region."""
    vals, vecs = np.linalg.eigh(cov)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    angle = float(np.degrees(np.arctan2(*vecs[:, 0][::-1])))
    w, h = 2.0 * n_std * np.sqrt(np.maximum(vals, 0.0))
    return Ellipse(xy=(mx, my), width=w, height=h, angle=angle, **kw)


def fig2_experiment(run_data: dict[tuple[int, str], list[list[dict]]]):
    """
    1x2 figure: left = Deterministic, right = pDSTL.

    Both panels use identical arena limits (_XLIM, _YLIM) so set_aspect('equal')
    produces the same physical panel size on both sides.

    Each panel shows:
      - Light-grey scatter of all actual Lighthouse samples (all 20 runs, all
        fan speeds pooled) — background spread of real flight positions.
      - Ensemble mean path per fan speed (plasma colormap), with 2σ empirical
        covariance ellipses drawn every 4 time bins (~1.2 s spacing).
        The ellipses are the cross-run 2×2 covariance of (x, y) at each bin.
      - Obstacle rectangles, start (^) and goal (*).

    Single figure-level legend below both panels.
    """
    ieee_style()

    cmap = plt.get_cmap('plasma')
    n_spd = max(len(FAN_SPEEDS), 1)
    speed_colour = {spd: cmap(i / (n_spd - 1) if n_spd > 1 else 0.0) for i, spd in enumerate(FAN_SPEEDS)}
    col_viol = '#d62728'

    fig, axes = plt.subplots(1, 2, figsize=(10.0, 6.5))
    handle_list: list = []
    label_list: list = []
    has_viols = False

    for ax, pt, panel_lbl in zip(
        axes,
        ['det', 'pdstl'],
        ['(a) Deterministic', '(b) pDSTL'],
    ):
        add_obstacles(ax)

        # Background scatter — pool all runs / all speeds for this condition
        all_rows = [r for spd in FAN_SPEEDS for runs in run_data.get((spd, pt), []) for r in runs]
        ax.scatter(
            [r['x'] for r in all_rows if r['safe']],
            [r['y'] for r in all_rows if r['safe']],
            s=2,
            color='#cccccc',
            alpha=0.15,
            zorder=3,
            rasterized=True,
        )
        viol_x = [r['x'] for r in all_rows if not r['safe']]
        viol_y = [r['y'] for r in all_rows if not r['safe']]
        if viol_x:
            ax.scatter(viol_x, viol_y, s=14, color=col_viol, marker='x', lw=1.0, alpha=0.70, zorder=4, rasterized=True)
            has_viols = True

        # Ensemble mean path + 2σ covariance ellipses per fan speed
        for spd in FAN_SPEEDS:
            runs = run_data.get((spd, pt), [])
            mx, my, cov_list = _mean_path_with_cov(runs)
            if mx:
                col = speed_colour[spd]
                # Draw 2σ ellipses at every 4th bin (~1.2 s spacing)
                for i in range(0, len(mx), 4):
                    e = _cov_ellipse_patch(
                        mx[i],
                        my[i],
                        cov_list[i],
                        n_std=2.0,
                        facecolor=col,
                        alpha=0.22,
                        edgecolor=col,
                        linewidth=0.6,
                        zorder=5,
                    )
                    ax.add_patch(e)
                (line,) = ax.plot(mx, my, '-', color=col, lw=2.0, zorder=6)
                if pt == 'det':  # collect legend handles once
                    handle_list.append(line)
                    label_list.append(f'Fan {spd}')

        ax.plot(*START, '^', color='#2ca02c', ms=7, zorder=7)
        ax.plot(*GOAL, '*', color='#d62728', ms=9, zorder=7)

        ax.set_xlim(*_XLIM)
        ax.set_ylim(*_YLIM)
        ax.set_aspect('equal')
        ax.set_title(panel_lbl, pad=4)
        ax.set_xlabel('$x$ (m)')
        ax.grid(True, alpha=0.25, lw=0.5)

    axes[0].set_ylabel('$y$ (m)')

    # Build shared legend: fan speed lines + Start / Goal / Violation proxies
    handle_list += [
        Line2D([], [], color='#2ca02c', marker='^', ls='None', ms=7),
        Line2D([], [], color='#d62728', marker='*', ls='None', ms=8),
    ]
    label_list += ['Start', 'Goal']
    if has_viols:
        handle_list.append(Line2D([], [], color=col_viol, marker='x', ls='None', ms=5, lw=1.0))
        label_list.append('Violation')

    fig.tight_layout()
    fig.subplots_adjust(bottom=0.13)
    fig.legend(
        handle_list,
        label_list,
        loc='lower center',
        ncol=len(handle_list),
        bbox_to_anchor=(0.5, 0.01),
        fontsize=14,
        handlelength=1.8,
        frameon=True,
        borderpad=0.6,
    )

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
        """Per-run minimum clearance (cm); violated/crashed runs contribute 0.0."""
        clrs = []
        for rows in runs:
            is_crash = bool(rows[0].get('_crash', False)) if rows else False
            violated = is_crash or (not rows) or any(not r['safe'] for r in rows)
            if violated:
                clrs.append(0.0)
            else:
                clrs.append(min(_min_obs_clearance(r['x'], r['y']) for r in rows))
        return np.array(clrs) if clrs else np.array([0.0])

    col_det = '#1f77b4'
    col_pdstl = '#2ca02c'

    clr_det_med, clr_det_q25, clr_det_q75 = [], [], []
    clr_pdstl_med, clr_pdstl_q25, clr_pdstl_q75 = [], [], []

    for spd in FAN_SPEEDS:
        for pt, med_list, q25_list, q75_list in [
            ('det', clr_det_med, clr_det_q25, clr_det_q75),
            ('pdstl', clr_pdstl_med, clr_pdstl_q25, clr_pdstl_q75),
        ]:
            runs = run_data.get((spd, pt), [])
            arr = _per_run_min_clr(runs)
            med_list.append(float(np.median(arr)))
            q25_list.append(float(np.percentile(arr, 25)))
            q75_list.append(float(np.percentile(arr, 75)))

    fig, ax = plt.subplots(figsize=(6.5, 4.5))
    for med, q25, q75, col, marker, lbl in [
        (clr_det_med, clr_det_q25, clr_det_q75, col_det, 'o', 'Deterministic'),
        (clr_pdstl_med, clr_pdstl_q25, clr_pdstl_q75, col_pdstl, 's', 'pDSTL'),
    ]:
        ax.plot(FAN_SPEEDS, med, f'{marker}-', color=col, lw=2.0, ms=5, label=lbl)
        ax.fill_between(FAN_SPEEDS, q25, q75, color=col, alpha=0.18)

    ax.axhline(0, color='#d62728', lw=1.0, ls='--', label='Obstacle boundary')
    ax.set_xlabel('Fan speed')
    ax.set_ylabel('Min. obstacle clearance (cm)')
    ax.set_title('Minimum obstacle clearance vs fan speed\n(median + IQR shading, 0 = violation)', pad=3)
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


def fig4_all_speeds(run_data: dict[tuple[int, str], list[list[dict]]]):
    """
    1x2 figure: ensemble mean paths ± 1-sigma band at all fan speeds.

    Identical arena limits and equal-weight per-run averaging as fig2.
    Legend labels use normalised values: spd / 1000
    (fan 0 → 0.0, fan 6 → 0.006, fan 12 → 0.012, fan 18 → 0.018).
    """
    ieee_style()

    cmap = plt.get_cmap('plasma')
    n = max(len(FAN_SPEEDS), 1)
    speed_colour = {spd: cmap(i / (n - 1) if n > 1 else 0.0) for i, spd in enumerate(FAN_SPEEDS)}

    fig, axes = plt.subplots(1, 2, figsize=(10.0, 6.5))
    handle_list: list = []
    label_list: list = []

    for ax, pt, panel_lbl in zip(
        axes,
        ['det', 'pdstl'],
        ['(a) Deterministic', '(b) pDSTL'],
    ):
        add_obstacles(ax)
        for spd in FAN_SPEEDS:
            runs = run_data.get((spd, pt), [])
            mx, my, cov_list = _mean_path_with_cov(runs)
            if mx:
                col = speed_colour[spd]
                for i in range(0, len(mx), 4):
                    e = _cov_ellipse_patch(
                        mx[i],
                        my[i],
                        cov_list[i],
                        n_std=2.0,
                        facecolor=col,
                        alpha=0.20,
                        edgecolor=col,
                        linewidth=0.6,
                        zorder=4,
                    )
                    ax.add_patch(e)
                (line,) = ax.plot(mx, my, '-', color=col, lw=2.0, zorder=5)
                if pt == 'det':  # collect legend handles once
                    handle_list.append(line)
                    label_list.append(f'Fan {spd}')

        ax.plot(*START, '^', color='#2ca02c', ms=7, zorder=6)
        ax.plot(*GOAL, '*', color='#d62728', ms=9, zorder=6)

        ax.set_xlim(*_XLIM)
        ax.set_ylim(*_YLIM)
        ax.set_aspect('equal')
        ax.set_title(panel_lbl, pad=4)
        ax.set_xlabel('$x$ (m)')
        ax.grid(True, alpha=0.25, lw=0.5)

    axes[0].set_ylabel('$y$ (m)')

    handle_list += [
        Line2D([], [], color='#2ca02c', marker='^', ls='None', ms=7),
        Line2D([], [], color='#d62728', marker='*', ls='None', ms=8),
    ]
    label_list += ['Start', 'Goal']

    fig.tight_layout()
    fig.subplots_adjust(bottom=0.13)
    fig.legend(
        handle_list,
        label_list,
        loc='lower center',
        ncol=len(handle_list),
        bbox_to_anchor=(0.5, 0.01),
        fontsize=14,
        handlelength=1.8,
        frameon=True,
        borderpad=0.6,
    )

    out = _OUT / 'fig4_all_speeds.pdf'
    fig.savefig(out, bbox_inches='tight')
    print(f'Saved {out}')
    plt.close(fig)


# =============================================================================
if __name__ == '__main__':
    RUN_DATA = load_all_runs(FAN_MAP, _CRASH_PATHS)

    if not RUN_DATA:
        print('[plot_results] No log files found in', _LOGS)
        print('  Expected: <condition>_fan<XX>_run<NN>_<ts>_actual.csv')
    else:
        print(f'[plot_results] Found {sum(len(v) for v in RUN_DATA.values())} runs across {len(RUN_DATA)} cells.')

    print_results_table(RUN_DATA)
    print_latex_table(RUN_DATA)
    fig1_paths()
    fig2_experiment(RUN_DATA)
    fig3_clearance(RUN_DATA)
    fig4_all_speeds(RUN_DATA)
    print(f'\nAll figures saved to {_OUT}/')

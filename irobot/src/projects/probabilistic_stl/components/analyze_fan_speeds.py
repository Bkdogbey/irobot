"""
Analysis script to calculate standard deviation and variance for logged data
in the 'deterministic' condition.

Calculates:
Statistics of the position error (Commanded - Actual) for each fan speed.
"""

from __future__ import annotations

import csv
import pathlib
import re

import numpy as np

# Path to logs directory (relative to this script)
LOGS_DIR = pathlib.Path(__file__).parent / 'logs'


def analyze_deterministic_logs():
    """
    Parses 'deterministic' logs, pairs actual vs commanded,
    and calculates statistics of the position error (Commanded - Actual).
    """
    if not LOGS_DIR.exists():
        print(f'Logs directory not found: {LOGS_DIR}')
        return

    # Regex to parse filename: deterministic_fanXX_runYY_..._actual.csv
    filename_pattern = re.compile(r'deterministic_fan(\d+)_run\d+_.*_actual\.csv')

    # Data storage: fan_speed -> list of error arrays (N, 3)
    errors_by_fan = {}

    print(f"Scanning {LOGS_DIR} for 'deterministic' logs...")

    files = list(LOGS_DIR.glob('deterministic_*_actual.csv'))
    if not files:
        print("No 'deterministic' actual logs found.")
        return

    for log_file in files:
        match = filename_pattern.search(log_file.name)
        if not match:
            continue

        fan_speed = int(match.group(1))

        # Construct commanded filename
        cmd_file = log_file.with_name(log_file.name.replace('_actual.csv', '_commanded.csv'))

        if not cmd_file.exists():
            continue

        if fan_speed not in errors_by_fan:
            errors_by_fan[fan_speed] = []

        try:
            # Read CSVs
            data_act = np.genfromtxt(log_file, delimiter=',', names=True, dtype=float)
            data_cmd = np.genfromtxt(cmd_file, delimiter=',', names=True, dtype=float)

            # Handle empty or single-point files
            if data_act.ndim == 0:
                data_act = np.atleast_1d(data_act)
            if data_cmd.ndim == 0:
                data_cmd = np.atleast_1d(data_cmd)

            if data_act.size < 2 or data_cmd.size < 1:
                continue

            # Extract columns
            t_act = np.atleast_1d(data_act['t'])
            pos_act = np.column_stack(
                (np.atleast_1d(data_act['x']), np.atleast_1d(data_act['y']), np.atleast_1d(data_act['z']))
            )

            t_cmd = np.atleast_1d(data_cmd['t'])
            pos_cmd = np.column_stack(
                (np.atleast_1d(data_cmd['x']), np.atleast_1d(data_cmd['y']), np.atleast_1d(data_cmd['z']))
            )

            # Match commanded to actual based on time (Zero Order Hold)
            # Find index in t_cmd such that t_cmd[i] <= t_act < t_cmd[i+1]
            indices = np.searchsorted(t_cmd, t_act, side='right') - 1

            # Filter valid matches (t_act must be >= first command time)
            valid_mask = (indices >= 0) & (indices < len(pos_cmd))

            if not np.any(valid_mask):
                continue

            matched_cmd = pos_cmd[indices[valid_mask]]
            matched_act = pos_act[valid_mask]

            # Calculate Error: Commanded - Actual
            error = matched_cmd - matched_act
            errors_by_fan[fan_speed].append(error)

        except Exception as e:
            print(f'Skipping {log_file.name}: {e}')

    # ── Statistics ───────────────────────────────────────────────────────────
    print('\n' + '=' * 80)
    print('STATISTICS OF POSITION ERROR (Commanded - Actual) [meters]')
    print('=' * 80)

    if not errors_by_fan:
        print('No valid data processed.')
        return

    for fan_speed in sorted(errors_by_fan.keys()):
        err_list = errors_by_fan[fan_speed]
        if not err_list:
            continue

        # Concatenate all runs
        all_errors = np.vstack(err_list)  # Shape (Total_Samples, 3)

        # Stats
        mean_err = np.mean(all_errors, axis=0)
        var_err = np.var(all_errors, axis=0, ddof=1)
        std_err = np.std(all_errors, axis=0, ddof=1)

        print(f'Fan Speed {fan_speed:02d}:')
        print(f'  Samples: {len(all_errors)}')
        print(f'  Mean Error (x, y, z): [{mean_err[0]:.4f}, {mean_err[1]:.4f}, {mean_err[2]:.4f}]')
        print(f'  Variance   (x, y, z): [{var_err[0]:.6f}, {var_err[1]:.6f}, {var_err[2]:.6f}]')
        print(f'  Std Dev    (x, y, z): [{std_err[0]:.4f}, {std_err[1]:.4f}, {std_err[2]:.4f}]')
        print('-' * 40)


if __name__ == '__main__':
    analyze_deterministic_logs()

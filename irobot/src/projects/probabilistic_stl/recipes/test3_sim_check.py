"""
TASK 5 — Simulation Check
Run this before every live flight. Must print PASS before touching hardware.
No CF hardware required or imported.
"""

import os
import sys
import torch
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..'))   # probabilistic_stl/

from planning.dynamics import SingleIntegrator
from planning.environment import build_test4_environment
from planning.planner import ProbabilisticSTLPlanner
from constraints.reach_only import build_reach_spec


T = 50
DT = 0.1
PLANNER_CFG = {
    'w_u': 0.5,
    'w_du': 0.05,
    'w_phi': 100.0,
    'lr': 0.05,
    'max_iters': 300,
    'alpha': 0.85,
    'w_dist': 5.0,
    'w_obs': 0.0,
    'w_visit': 0.0,
}


def main():
    print('=== TASK 5: Simulation Check ===')

    env = build_test4_environment()
    dynamics = SingleIntegrator(dt=DT, u_max=0.3, q_std=0.05)
    planner = ProbabilisticSTLPlanner(dynamics, env, T, config=PLANNER_CFG)
    spec = build_reach_spec(env, T)

    x0_mean = torch.tensor([0.0, 0.0])
    x0_cov = torch.eye(2) * 0.01

    print(f'Running optimizer (T={T} steps = {T * DT:.1f}s trajectory)...')
    mean_trace, cov_trace, u_trace, p_sat, history = planner.solve(x0_mean, x0_cov, verbose=True, spec=spec)

    # Checks
    final_pos = mean_trace[0, -1, :].numpy()
    goal_x = env.goal['x']
    goal_y = env.goal['y']
    in_goal = goal_x[0] <= final_pos[0] <= goal_x[1] and goal_y[0] <= final_pos[1] <= goal_y[1]

    print(f'\n--- Results ---')
    print(f'P(satisfaction) = {p_sat:.4f}   (required >= 0.85)')
    print(f'Final position  = [{final_pos[0]:.3f}, {final_pos[1]:.3f}]')
    print(f'Goal region     = x{goal_x}  y{goal_y}')
    print(f'In goal region  = {in_goal}')

    # Plot
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches

        fig, ax = plt.subplots(figsize=(8, 5))
        path = mean_trace[0].detach().numpy()
        ax.plot(path[:, 0], path[:, 1], 'b-o', markersize=3, label='Planned path')
        ax.add_patch(
            patches.Rectangle(
                (goal_x[0], goal_y[0]),
                goal_x[1] - goal_x[0],
                goal_y[1] - goal_y[0],
                color='green',
                alpha=0.4,
                label='Goal',
            )
        )
        ax.scatter([0], [0], color='red', s=100, zorder=5, label='Start')
        ax.set_xlim(-0.5, 2.5)
        ax.set_ylim(-1.0, 1.0)
        ax.set_aspect('equal')
        ax.legend()
        ax.set_title(f'Simulation Check | P(sat)={p_sat:.4f}')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig('sim_check_result.png', dpi=150)
        print('Plot saved to sim_check_result.png')
        plt.show()
    except Exception as e:
        print(f'Plot failed (non-fatal): {e}')

    # Pass/Fail
    if p_sat >= 0.85 and in_goal:
        print('\n=== TASK 5 PASS — Safe to proceed to live flight ===')
    else:
        print('\n=== TASK 5 FAIL — DO NOT FLY ===')
        if p_sat < 0.85:
            print(f'  P(sat)={p_sat:.4f} below 0.85 — tune planner config')
        if not in_goal:
            print(f'  Final position outside goal — check goal region or T')
        sys.exit(1)


if __name__ == '__main__':
    main()

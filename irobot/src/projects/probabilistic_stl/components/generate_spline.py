"""
Generates a Cubic Spline trajectory for the STL mission.

Mission Geometry (from stl_mission.py):
  Start:    (-1.0,  1.0)
  Workspace: x in [-1, 1], y in [-2, 2]
  Obstacle:  x = [0.00, 0.75],  y = [-0.75, 1.00]
  Goal:      x = [0.70, 1.00],  y = [-2.00, -1.70]
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline


def get_obstacles():
    """Return list of obstacles as (x_min, x_max, y_min, y_max)."""
    return [
        (0.00, 0.35, -1.00, -0.75),  # Obstacle 1
        (0.50, 0.85, -1.50, -1.25),  # Obstacle 2
    ]


def get_waypoints():
    """Return list of waypoints as (t, x, y, z)."""
    return [
        # t,    x,     y,    z
        (0.0, -1.0, 1.0, 0.5),  # Start (Hover height 0.5m)
        (4.0, -0.8, -0.5, 0.5),  # Drop down, stay left
        (7.0, -0.2, -1.6, 0.5),  # Go below both obstacles (y < -1.5)
        (10.0, 1.0, -2.0, 0.5),  # Goal
    ]


def generate_trajectory(n_points: int = 100) -> list[tuple[float, float, float]]:
    """Generate the trajectory points from the defined waypoints."""
    waypoints = get_waypoints()
    data = np.array(waypoints)
    t_points = data[:, 0]
    x_points = data[:, 1]
    y_points = data[:, 2]
    z_points = data[:, 3]

    # 2. Create Cubic Splines for each axis
    cs_x = CubicSpline(t_points, x_points)
    cs_y = CubicSpline(t_points, y_points)
    cs_z = CubicSpline(t_points, z_points)

    times = np.linspace(t_points[0], t_points[-1], n_points)

    return [(float(cs_x(t)), float(cs_y(t)), float(cs_z(t))) for t in times]


def main():
    import matplotlib.pyplot as plt

    obstacles = get_obstacles()
    waypoints = get_waypoints()

    # Generate trajectory using the shared function
    traj = generate_trajectory(n_points=100)
    traj_x = [p[0] for p in traj]
    traj_y = [p[1] for p in traj]
    x_points = [w[1] for w in waypoints]
    y_points = [w[2] for w in waypoints]

    # 4. Visualize to verify obstacle avoidance
    plt.figure(figsize=(6, 6))

    # Plot Obstacles
    for i, (x1, x2, y1, y2) in enumerate(obstacles):
        label = 'Obstacle' if i == 0 else None
        plt.fill_between([x1, x2], y1, y2, color='red', alpha=0.3, label=label)

    # Plot Goal (clamped to workspace)
    plt.fill_between([0.70, 1.00], -2.00, -1.70, color='green', alpha=0.3, label='Goal')

    # Plot Workspace Limits
    plt.plot([-1, 1, 1, -1, -1], [-2, -2, 2, 2, -2], 'k--', linewidth=1, label='Workspace')

    # Plot Path
    plt.plot(traj_x, traj_y, 'b-', linewidth=2, label='Spline Path')
    plt.plot(x_points, y_points, 'ko', label='Waypoints')

    plt.grid(True)
    plt.legend()
    plt.title('Drone Spline Trajectory')
    plt.show()


if __name__ == '__main__':
    main()

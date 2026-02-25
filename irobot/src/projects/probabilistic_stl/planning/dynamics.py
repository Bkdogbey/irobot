import torch
import torch.nn as nn


class Dynamics(nn.Module):
    """
    Base class for system dynamics.
    Handles control bounding and common initialization.
    """

    def __init__(self, dt, u_max, device="cpu"):
        super().__init__()
        self.dt = dt
        self.u_max = u_max
        self.device = device

    def bound_control(self, v):
        """
        Applies smooth squashing to keep control within [-u_max, u_max].
        v: Unconstrained optimization variable (the 'knobs' for the optimizer)
        u: Physical control input applied to the robot
        """
        return self.u_max * torch.tanh(v)

    def step(self, x, P, u):
        """
        Propagates state x and covariance P one step forward with control u.
        x: [Dim]
        P: [Dim, Dim]
        u: [Control Dim]
        Returns: (x_next, P_next)
        """
        raise NotImplementedError

    def forward(self, v_sequence, x0_mean, x0_cov):
        raise NotImplementedError


class SingleIntegrator(Dynamics):
    """
    Standard Position-Velocity model.

    State:   [x, y]
    Control: [vx, vy]

    mu_{t+1}    = mu_t + u_t * dt
    Sigma_{t+1} = Sigma_t + Q     where Q = diag(q_std^2)
    """

    def __init__(self, dt=0.1, u_max=0.3, q_std=0.05, device="cpu"):
        super().__init__(dt, u_max, device)
        self.Q = torch.eye(2, device=self.device) * q_std**2

    def step(self, x, P, u):
        return x + u * self.dt, P + self.Q

    def forward(self, v_sequence, x0_mean, x0_cov):
        """
        Rolls out the trajectory from t=0 to T.

        Args:
            v_sequence: Tensor [T, 2] (Unconstrained controls)
            x0_mean:    Tensor [2]    (Initial position)
            x0_cov:     Tensor [2, 2] (Initial uncertainty)

        Returns:
            mean_stack: [1, T+1, 2]
            cov_stack:  [1, T+1, 2, 2]
        """
        T = v_sequence.shape[0]

        means = [x0_mean]
        covs = [x0_cov]

        curr_mu = x0_mean
        curr_sigma = x0_cov

        for t in range(T):
            u = self.bound_control(v_sequence[t])
            curr_mu = curr_mu + u * self.dt
            curr_sigma = curr_sigma + self.Q
            means.append(curr_mu)
            covs.append(curr_sigma)

        mean_stack = torch.stack(means).unsqueeze(0)
        cov_stack = torch.stack(covs).unsqueeze(0)

        return mean_stack, cov_stack


class GaussianBelief2D:
    """
    Wraps a single timestep from the dynamics rollout.
    mean_full: [Batch, 2] tensor
    var_full:  [Batch, 2, 2] tensor (full covariance)
    """

    def __init__(self, mean_full, var_full):
        self.mean_full = mean_full
        self.var_full  = var_full

    def value(self):
        return self.mean_full

    def probability_of(self, residual):
        raise NotImplementedError(
            "Use RectangularGoalPredicate / RectangularObstaclePredicate "
            "which access mean_full and var_full directly."
        )

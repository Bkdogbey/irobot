import time
import torch
import logging

logger = logging.getLogger(__name__)


class CrazyflieSTLExecutor:
    """
    Bridges pdSTL planner output to CrazyflieComponent hardware commands.
    Experiment-specific — not a general driver.
    Assumes 2D navigation at fixed hover height via send_hover_setpoint.
    """

    def __init__(self, cf_component, dt=0.1, z_hold=0.5,
                 q_std=0.05, u_max=0.3):
        self.cf      = cf_component
        self.dt      = dt
        self.z_hold  = z_hold

        from planning.dynamics import SingleIntegrator
        self.dynamics = SingleIntegrator(dt=dt, u_max=u_max, q_std=q_std)

    def wait_for_state(self, timeout=5.0):
        """
        Block until lighthouse state is available.
        Raises RuntimeError if timeout exceeded.
        """
        start = time.time()
        while not self.cf.state_ready:
            if time.time() - start > timeout:
                raise RuntimeError(
                    "Lighthouse state not received within "
                    f"{timeout}s. Check Task 1A (geometry calibration)."
                )
            time.sleep(0.05)
        logger.info('Lighthouse state ready: x=%.3f y=%.3f z=%.3f',
                    self.cf.current_x, self.cf.current_y, self.cf.current_z)

    def get_belief(self, inflate_cov=0.02):
        """
        Build Gaussian belief from current lighthouse state.
        inflate_cov: initial position uncertainty in m^2 (~14cm 1-sigma for lighthouse)
        """
        if not self.cf.state_ready:
            raise RuntimeError("get_belief called before state_ready")
        mean = torch.tensor(
            [self.cf.current_x, self.cf.current_y], dtype=torch.float32
        )
        cov = torch.eye(2, dtype=torch.float32) * inflate_cov
        return mean, cov

    def plan(self, env, T, planner_cfg, spec_fn, verbose=True):
        """
        Run pdSTL optimizer from current lighthouse position.
        spec_fn: callable(env, T) -> STL_Formula
        Returns: u_trace, mean_trace, cov_trace, p_sat, history
        """
        from planning.planner import ProbabilisticSTLPlanner

        x0_mean, x0_cov = self.get_belief()
        spec = spec_fn(env, T)

        planner = ProbabilisticSTLPlanner(
            self.dynamics, env, T, config=planner_cfg
        )
        mean_trace, cov_trace, u_trace, p_sat, history = planner.solve(
            x0_mean, x0_cov, verbose=verbose, spec=spec
        )

        logger.info('Plan complete | P(sat)=%.4f', p_sat)
        return u_trace, mean_trace, cov_trace, p_sat, history

    def execute_open_loop(self, u_trace):
        """
        Execute pre-planned trajectory open-loop.
        Iterates through u_trace sending each [vx, vy] as a hover setpoint.
        Sends zero velocity for 1 second after completion.
        """
        u_np = u_trace.detach().cpu().squeeze().numpy()
        logger.info('Executing open-loop trajectory: %d steps x %.2fs = %.1fs',
                    len(u_np), self.dt, len(u_np) * self.dt)

        for u in u_np:
            vx = float(u[0])
            vy = float(u[1])
            self.cf.send_velocity_setpoint(vx, vy, 0.0, self.z_hold)
            time.sleep(self.dt)

        self._stop()

    def execute_mpc(self, env, T_horizon, planner_cfg, spec_fn,
                    max_steps=150, goal_tol=0.25):
        """
        Receding horizon MPC: replan every step from live lighthouse position.
        Use only after open-loop (Task 6) validates the pipeline.
        """
        goal_center = torch.tensor([
            sum(env.goal["x"]) / 2.0,
            sum(env.goal["y"]) / 2.0,
        ])

        for step in range(max_steps):
            curr_mean, _ = self.get_belief()
            dist = torch.norm(curr_mean - goal_center).item()

            logger.info('MPC Step %03d | pos=[%.2f, %.2f] | dist=%.2f | goal=[%.2f, %.2f]',
                        step, curr_mean[0], curr_mean[1], dist,
                        goal_center[0], goal_center[1])

            if dist < goal_tol:
                logger.info('Goal reached at step %d', step)
                break

            u_trace, _, _, p_sat, _ = self.plan(
                env, T_horizon, planner_cfg, spec_fn, verbose=False
            )
            u0 = u_trace[0].detach().cpu().numpy()
            self.cf.send_velocity_setpoint(
                float(u0[0]), float(u0[1]), 0.0, self.z_hold
            )
            time.sleep(self.dt)

        self._stop()

    def _stop(self, duration=1.0):
        """Send zero velocity setpoints to halt the drone."""
        steps = int(duration / 0.05)
        for _ in range(steps):
            self.cf.send_velocity_setpoint(0.0, 0.0, 0.0, self.z_hold)
            time.sleep(0.05)

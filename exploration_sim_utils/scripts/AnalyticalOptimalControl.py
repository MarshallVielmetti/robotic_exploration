"""
@file AnalyticalOptimalControl.py
@brief Implements the analytical optimal control solver
for the CarDynamics system

Does not take into account control constraints
"""

from State import State, Control
from CarDynamics import CarDynamics

from AnalyticalControlUtil import AnalyticalControlUtil

import numpy as np
from scipy.linalg import expm
from scipy.integrate import quad_vec


class AnalyticalOptimalControlSolver:
    def __init__(self, car_dynamics: CarDynamics):
        self.car_dynamics = car_dynamics

        # State Cost
        self.Q = np.diag([0.0, 0.0, 0.0, 1.0, 1.0])

        # Control Cost
        self.R = np.diag([1.0, 1.0])

        # penalty on final time
        self.lambda_t = 1.0

    def solve(
        self, x0: State, xg: State, N: int
    ) -> tuple[float, list[State], list[Control]]:
        """
        Solves the optimal control problem using the analytical solution
        of the linear quadratic regulator

        @param x0: The initial state
        @param xg: The goal state
        @param N: The number of time steps to simulate
        @return: The optimal cost and the optimal trajectory
        """

        x0 = np.array([x0.x, x0.y, x0.theta, x0.v, x0.kappa])
        xg = np.array([xg.x, xg.y, xg.theta, xg.v, xg.kappa])

        A, B, c = self.car_dynamics.linearize(xg)

        # Compute tau star
        R_inv = np.linalg.inv(self.car_dynamics.R)
        tau_star = AnalyticalControlUtil.tau_star(A, B, c, R_inv, x0, xg)

        composite_matrix = np.block([[A, B @ R_inv @ B.T], [np.zeros_like(A), -A.T]])

        d_tau_star = AnalyticalControlUtil.get_d_of_tau(
            A, B, c, R_inv, x0, xg, tau_star
        )

        comp_inv = np.linalg.pinv(composite_matrix)
        c_zeros = np.concatenate([c, np.zeros_like(c)]).reshape(-1, 1)
        integral_func = (
            lambda t: comp_inv
            @ (
                AnalyticalControlUtil.fast_expm(composite_matrix, t)
                - np.eye(2 * A.shape[0])
            )
            @ c_zeros
        )

        # x1 stacked with d[tau*] -- Eq. (20)
        x1_dtau_star = np.concatenate([xg, d_tau_star]).reshape(-1, 1)

        composite_state = []
        for t in np.arange(0, tau_star, 0.1):

            no_control_solution = (
                AnalyticalControlUtil.fast_expm(composite_matrix, (t - tau_star))
                @ x1_dtau_star
            )

            # no state solution

            # no_state_solution, err = quad_vec(
            #     lambda t_prime: AnalyticalControlUtil.fast_expm(
            #         composite_matrix, (t - t_prime)
            #     )
            #     @ c_zeros,
            #     tau_star,
            #     t,
            # )

            forced_response = integral_func(t - tau_star)

            composite_state.append(no_control_solution + forced_response)

        states = []
        controls = []
        for state in composite_state:
            states.append(State(state[0], state[1], state[2], state[3], state[4]))

            # Extract the control from the state
            # Recover input using Eq. (16)
            ctr = R_inv @ B.T @ state[5:]
            controls.append(Control(ctr[0], ctr[1]))

        return tau_star, states, controls

"""
@file NumericalOptimalControl.py
@brief Implements the numerical optimal control solver
for the CarDynamics system
"""

from State import State, Control
import numpy as np

import cvxpy as cp


class NumericalOptimalControlSolver:
    def __init__(self, car_dynamics):
        self.car_dynamics = car_dynamics

        self.input_constraints = {
            "u_min": np.array([-1.0, -0.5]),
            "u_max": np.array([1.0, 0.5]),
        }

        self.state_constraints = {
            "x_min": np.array([-np.inf, -np.inf, -np.inf, 0.01, -0.5]),
            "x_max": np.array([np.inf, np.inf, np.inf, 10, 0.5]),
        }

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
        Solves the optimal control problem using a
        convex optimization formulation of the
        direct shooting method

        @param x0: The initial state
        @param xg: The goal state
        @param N: The number of time steps to simulate
        @return: The optimal cost and the optimal trajectory
        """

        x0 = np.array([x0.x, x0.y, x0.theta, x0.v, x0.kappa])
        xg = np.array([xg.x, xg.y, xg.theta, xg.v, xg.kappa])

        min_cost = np.inf
        best_states = None
        best_controls = None

        T_guess = 20.0
        for iteration in range(5):
            cost, states, controls = self.solve_fixed_time(x0, xg, N, T_guess)

            if cost is None:
                continue

            if cost < min_cost:
                min_cost = cost
                best_states = states
                best_controls = controls

            T_guess = 0.5 * T_guess

        if best_states is None:
            return None, None, None

        # Convert to types
        best_states = [State(x[0], x[1], x[2], x[3], x[4]) for x in best_states]
        best_controls = [Control(u[0], u[1]) for u in best_controls]

        return min_cost, best_states, best_controls

    def solve_fixed_time(
        self, x0: State, xg: State, N, T_guess
    ) -> tuple[float, list[State], list[Control]]:
        """
        Solves the optimal control problem with a fixed final time
        """
        A, B, c = self.car_dynamics.linearize(xg)

        n = A.shape[0]
        m = B.shape[1]

        X = cp.Variable((n, N + 1))  # State trajectory
        U = cp.Variable((m, N))  # Control trajectory
        s_terminal = cp.Variable(n)

        lambda_terminal = 100.0  # slack penalty on terminal constraint

        constraints = []
        objective = cp.sum([cp.quad_form(U[:, k], self.R) for k in range(N)])
        objective += T_guess * self.lambda_t
        objective += lambda_terminal * cp.sum_squares(s_terminal)

        for k in range(N):
            # Shooting constriants
            constraints.append(
                X[:, k + 1] == X[:, k] + (T_guess / N) * (A @ X[:, k] + B @ U[:, k] + c)
            )

            # input constraints
            constraints.append(self.input_constraints["u_min"] <= U[:, k])
            constraints.append(U[:, k] <= self.input_constraints["u_max"])

            # state constraints
            constraints.append(self.state_constraints["x_min"][3:] <= X[3:, k])
            constraints.append(X[3:, k] <= self.state_constraints["x_max"][3:])

        # Initial and final state constraints
        constraints.append(X[:, 0] == x0)
        constraints.append(X[:, N] == xg + s_terminal)

        # bound the slack variable
        constraints.append(s_terminal >= -5.0)
        constraints.append(s_terminal <= 5.0)

        # final time constraints
        # constraints.append(T >= 0)

        # Solve the problem
        problem = cp.Problem(cp.Minimize(objective), constraints)

        try:
            # problem.solve(solver=cp.ECOS, max_iters=200, abstol=1e-6, reltol=1e-6)
            # problem.solve(solver=cp.ECOS, abstol=1e-2, reltol=1e-2, max_iters=200)
            problem.solve(
                solver=cp.OSQP,
                eps_abs=1e-1,
                eps_rel=1e-1,
                max_iter=200,
                warm_start=True,
            )

            if problem.status not in ["optimal"]:
                # print(f"Optimization Failed -- status {problem.status}")
                raise Exception(f"Optimization status: {problem.status}")

            # print(f"Optimization Successful! -- {problem.value}")

            states = [X[:, k].value for k in range(N + 1)]
            controls = [U[:, k].value for k in range(N)]

            # normalize theta
            for k in range(N + 1):
                states[k][2] = (states[k][2] + np.pi) % (2 * np.pi) - np.pi

            return problem.value, states, controls

        except Exception as e:
            print(f"Optimization Failed -- {e}")
            return None, None, None

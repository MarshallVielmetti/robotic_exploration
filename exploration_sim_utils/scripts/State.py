import numpy as np
from scipy.integrate import solve_ivp
import scipy.integrate as spi
from scipy.optimize import minimize
from scipy.linalg import expm

import cvxpy as cp


def get_random_state(x_min, x_max, y_min, y_max):
    """
    Returns a random state in the given bounds
    """
    x = np.random.uniform(x_min, x_max)
    y = np.random.uniform(y_min, y_max)
    theta = np.random.uniform(-np.pi, np.pi)
    v = np.random.uniform(State.V_MIN, State.V_MAX)
    kappa = np.random.uniform(State.KAPPA_MIN, State.KAPPA_MAX)

    return State(x, y, theta, v, kappa)


class State:
    V_MIN: float = 0.001
    V_MAX: float = 10.0
    KAPPA_MIN: float = -0.5
    KAPPA_MAX: float = 0.5

    def __init__(self, x=0, y=0, theta=0, v=0.1, kappa=0):
        self.x: float = x
        self.y: float = y
        self.theta: float = theta
        self.v: float = v
        self.kappa: float = kappa

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta}, {self.v}, {self.kappa})"

    def __hash__(self):
        return hash(
            (
                round(self.x, 4),
                round(self.y, 4),
                round(self.theta, 4),
                round(self.v, 4),
                round(self.kappa, 4),
            )
        )

    def __eq__(self, other):
        if not isinstance(other, State):
            return False

        return (
            abs(self.x - other.x) < 1e-4
            and abs(self.y - other.y) < 1e-4
            and abs(self.theta - other.theta) < 1e-4
            and abs(self.v - other.v) < 1e-4
            and abs(self.kappa - other.kappa) < 1e-4
        )


class Control:
    """
    Control class to represent the control input
    """

    def __init__(self, a, alpha):
        self.control = np.array([a, alpha])

    def __str__(self):
        return f"({self.control[0]}, {self.control[1]})"

    def a(self):
        return self.control[0]

    def alpha(self):
        return self.control[1]


def compute_gramian(A, B, R, tau):
    """
    Computes the weighted controllability gramian by finding the solution to
    the Lyapunov equation
    """
    n = A.shape[0]
    BRinvB_t = B @ np.linalg.inv(R) @ B.T  # precompute B @ R^-1 @ B^T

    def lyapunov_ode(t, G_flat):
        G = G_flat.reshape(n, n)
        dG_dt = A @ G + G @ A.T + BRinvB_t
        return dG_dt.flatten()

    # solve from G(0) = 0
    G0 = np.zeros((n, n))
    sol = solve_ivp(lyapunov_ode, [0, tau], G0.flatten(), method="RK45")
    G_tau = sol.y[:, -1].reshape(n, n)
    return G_tau


def compute_xbar(A, c, tau):
    """
    Computes the free evolution of the state without control input
    by solving the differential equation:
    xbar_dot = A @ xbar + c, xbar(0) = 0
    """
    n = A.shape[0]

    def xbar_ode(t, xbar):
        return A @ xbar + c

    xbar0 = np.zeros(n)
    sol = solve_ivp(xbar_ode, [0, tau], xbar0, method="RK45")
    return sol.y[:, -1]


def compute_optimal_control_policy(A, B, R, x1, t, tau, xbar_tau, G_tau):
    """
    Computes u[t]
    """
    r_inv = np.linalg.inv(R)
    G_inv = np.linalg.inv(G_tau)
    eAt_T = expm(A.T * (tau - t))

    x_diff = x1 - xbar_tau

    return r_inv @ B.T @ eAt_T @ G_inv @ x_diff


def get_d_tau(A, B, c, R, x1, tau):
    G_tau = compute_gramian(A, B, R, tau)
    x_bar_tau = compute_xbar(A, c, tau)

    d_tau = np.linalg.inv(G_tau) @ (x1 - x_bar_tau)
    return d_tau


def find_tau_star(A, B, c, R, x1):
    R_inv = np.linalg.inv(R)
    B_R_inv_B_T = B @ R_inv @ B.T  # precompute for efficienty
    a_x1_c = A @ x1 + c

    # def cost(tau):
    #     G_tau = compute_gramian(A, B, R, tau)
    #     x_bar_tau = compute_xbar(A, c, tau)
    #     return tau + (x1 - x_bar_tau).T @ G_tau @ (x1 - x_bar_tau)

    def cost_ode(tau):
        d_tau = get_d_tau(A, B, c, R, x1, tau)

        return 1 - 2 * (a_x1_c).T @ d_tau - d_tau.T @ B_R_inv_B_T @ d_tau

    # solve cdot[tau] = 0 for tau
    res = minimize(cost_ode, 0.1, bounds=[(0.001, None)])
    return res.x[0]


class CarDynamics:
    def __init__(self, w_v=1.0, w_kappa=1.0):
        self.R = np.diag([w_v, w_kappa])

        self.input_constraints = {
            "u_min": np.array([-1.0, -0.5]),
            "u_max": np.array([1.0, 0.5]),
        }

        self.state_constraints = {
            "x_min": np.array([-np.inf, -np.inf, -np.inf, 0, -0.5]),
            "x_max": np.array([np.inf, np.inf, np.inf, 10, 0.5]),
        }

    def linearaize(self, x_hat):
        """
        Linearize the dynamics around state x_hat
        Returns matrices A, B, c, xdot = Ax + Bu + c
        """
        x, y, theta, v, kappa = x_hat

        A = np.array(
            [
                [0, 0, -v * np.sin(theta), np.cos(theta), 0],
                [0, 0, v * np.cos(theta), np.sin(theta), 0],
                [0, 0, 0, kappa, v],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
            ]
        )

        B = np.array([[0, 0], [0, 0], [0, 0], [1, 0], [0, 1]])

        c = (
            np.array([v * np.cos(theta), v * np.sin(theta), v * kappa, 0, 0])
            - A @ x_hat
        )

        return A, B, c

    def discretized_dynamics(self, A, B, c, tau, dt):
        """
        Discretize the linearized dynamics
        """
        n = A.shape[0]
        m = B.shape[1]

        # state transition
        Ad = expm(A * dt)

        # input -- numerical integration
        Bd = np.zeros((n, m))
        for i in range(100):
            tau = dt * i / 100
            Bd += dt / 100 * expm(A * (dt - tau)) @ B

        # For the constant term
        cd = np.zeros(n)
        for i in range(100):
            tau = dt * i / 100.0
            cd += dt / 100.0 * expm(A * (dt - tau)) @ c

        return Ad, Bd, cd

    def compute_constrained_cost(self, x0, x1, N=20):
        """
        Computes optimal trajectory from x0 to x1 with input and state constraints
        discretized into N steps
        """
        print("Computing Constrained Cost")

        # Linearize around x0
        A, B, c = self.linearaize(x0)

        # Estimate time to reach
        tau_star = find_tau_star(A, B, c, self.R, x1)
        dt = tau_star * 1000 / N

        # discretize dynamics
        Ad, Bd, cd = self.discretized_dynamics(A, B, c, tau_star, dt)

        # Define optimization variables
        x = [cp.Variable(5) for _ in range(N + 1)]
        u = [cp.Variable(2) for _ in range(N)]

        # Objective minimize control effort and time
        objective = cp.sum([cp.quad_form(u[k], self.R) for k in range(N)]) + tau_star

        # Constraints
        constraints = [x[0] == x0]
        for k in range(N):
            # Dynamics Constraints
            constraints.append(x[k + 1] == Ad @ x[k] + Bd @ u[k] + cd)

            # Input constraints
            constraints.append(u[k] >= self.input_constraints["u_min"])
            constraints.append(u[k] <= self.input_constraints["u_max"])

            # state constraints
            constraints.append(x[k + 1] >= self.state_constraints["x_min"])
            constraints.append(x[k + 1] <= self.state_constraints["x_max"])

        slack = cp.Variable(5, nonneg=True)
        terminal_weight = 1000

        # Add terminal constraint
        constraints.append(x[N] - x1 <= slack)
        constraints.append(x[N] - x1 >= -slack)

        objective += terminal_weight * cp.sum(slack)

        # define and solve problem
        problem = cp.Problem(cp.Minimize(objective), constraints)

        try:
            problem.solve(solver=cp.OSQP, verbose=True)

            if problem.status in ["infeasible", "unbounded"]:
                print(f"Optimization Failed -- status {problem.status}")
                raise Exception(f"Optimization status: {problem.status}")

            states = [x[k].value for k in range(N + 1)]
            controls = [u[k].value for k in range(N)]

            print(f"Optimization Successful!")

            return tau_star, states, controls
        except Exception as e:
            print(f"Optimization Failed - {e}")
            return None, None, None

    def compute_cost(self, x0, x1):
        """
        Compute the optimal arrival time T* and cost c[tau] between x0 and x1
        using the linearized system dynamics
        """
        A, B, c = self.linearaize(x0)

        tau_star = find_tau_star(A, B, c, self.R, x1)

        R_inv = np.linalg.inv(self.R)

        composite_matrix = np.block([[A, B @ R_inv @ B.T], [np.zeros_like(A), -A.T]])

        d_tau_star = get_d_tau(A, B, c, self.R, x1, tau_star)

        composite_state = []

        # Compute the no control compontent
        for t in np.arange(0, tau_star, 0.01):

            x_1_d_tau_star = np.concatenate([x1, d_tau_star]).reshape(-1, 1)
            print(f"Shape of x_1_d_tau_star: {x_1_d_tau_star.shape}")

            no_control_solution = (
                expm(composite_matrix * (t - tau_star)) @ x_1_d_tau_star
            )

            # no state solution
            c_zeros = np.concatenate([c, np.zeros_like(c)]).reshape(-1, 1)
            print(f"Shape of c_zeros: {c_zeros.shape}")

            print(f"Shape of t: {t}")
            print(f"Shape of tau_star: {tau_star}")

            no_state_solution, err = spi.quad_vec(
                lambda t_prime: expm(composite_matrix * (t - t_prime)) @ c_zeros,
                tau_star,
                t,
            )

            print(f"Shape of no_state_solution: {no_state_solution.shape}")

            composite_state.append(no_control_solution + no_state_solution)

        # Break the composite state into the state and control components
        states = []
        controls = []
        for state in composite_state:
            states.append(state[:5])
            controls.append(R_inv @ B.T @ state[5:])

        return tau_star, states, controls

    def estimate_cost(self, x0, x1):
        """
        Fast analytical cost estimation between two states
        Used for RRT nearest neighbor and rewiring
        """

        # Linearize around x0
        A, B, c = self.linearaize(x0)

        try:
            # Compute optimal time -- faster than full trajecotry
            tau_star = find_tau_star(A, B, c, self.R, x1)

            # Add a heuristic penalty for states near constraints
            penalty = 0.0

            if x0[3] < 0.05 or x1[3] < 0.05:
                # Low velocity
                penalty += 5.0

            if abs(x0[4] > 0.4 or abs(x1[4]) > 0.4):
                penalty += 2.0

            return tau_star + penalty
        except Exception as e:
            return 1000.0

"""
@file AnalyticalControlUtil.py
@brief Utility functions used by the AnalyticalOptimalControlSolver
"""

import numpy as np

from scipy.linalg import solve_continuous_lyapunov, expm
from scipy.integrate import quad_vec
from scipy.optimize import fsolve


class AnalyticalControlUtil:

    @staticmethod
    def tau_star(A, B, c, R_inv, x0, x1):
        """
        Finds tau_star by evaluating the cost function
        at the zeros of the derivative of the cost function,
        given by Eq. (13) in the paper
        """

        def cost_function(tau):
            """
            Implements Eq. (11) from the paper
            """
            xbar_tau = AnalyticalControlUtil.get_xbar_of_tau(A, B, c, R_inv, x0, tau)
            g_tau = AnalyticalControlUtil.get_G_of_tau(A, B, c, R_inv, tau)
            g_tau_inv = np.linalg.inv(g_tau)

            return tau + (x1 - xbar_tau).T @ g_tau_inv @ (x1 - xbar_tau)

        def cost_ode(tau):
            """
            Implements Eq. (13) from the paper
            """
            d_tau = AnalyticalControlUtil.get_d_of_tau(A, B, c, R_inv, x0, x1, tau)

            return 1 - 2 * (A @ x1 + c).T @ d_tau - d_tau.T @ B @ R_inv @ B.T @ d_tau

        critical_points = fsolve(cost_ode, 0.1)

        # Find the tau* that minimizes the cost function

        min_tau_star = np.inf
        min_cost = np.inf

        for critical_point in critical_points:
            cost = cost_function(critical_point)
            if cost < min_cost:
                min_cost = cost
                min_tau_star = critical_point

        # returns the tau* that minimizes the cost function
        return min_tau_star

    @staticmethod
    def get_d_of_tau(A, B, c, R_inv, x0, x1, tau):
        """
        Implements Eq. (14) from the paper

        d[tau] = G[tau]^{-1} @ (x1 - x_bar[tau])
        """
        G_tau = AnalyticalControlUtil.get_G_of_tau(A, B, c, R_inv, tau)

        G_tau_inv = np.linalg.inv(G_tau)
        xbar_tau = AnalyticalControlUtil.get_xbar_of_tau(A, B, c, R_inv, x0, tau)

        return G_tau_inv @ (x1 - xbar_tau)

    @staticmethod
    def get_G_of_tau(A, B, c, R_inv, tau):
        """
        Solves for G(t) by finding a solution to the Lyapunov equation
        given in Eq. (7)
        """
        return solve_continuous_lyapunov(A, -B @ R_inv @ B.T)

        def lyapunov_eq(G):
            """
            Implements Eq. (7) from the paper
            """
            return A @ G + G @ A.T + B @ R_inv @ B.T

        return np.linalg.solve_lyapunov(lyapunov_eq, -c @ c.T)

    @staticmethod
    def get_xbar_of_tau(A, B, c, R_inv, x0, tau):
        """
        Xbar of tau represente the value of the state
        at time tau if no control is applied

        Given by Eq. (8) in the paper
        """
        return (
            AnalyticalControlUtil.fast_expm(A, tau) @ x0
            + quad_vec(
                lambda t: AnalyticalControlUtil.fast_expm(A, (tau - t)) @ c, 0, tau
            )[0]
        )

    @staticmethod
    def fast_expm(A, tau):
        """
        Fast matrix exponential calculation for nilpotent A
        """
        if tau > 5:
            return np.zeros_like(A)

        return expm(A * tau)

"""
File: CarDynamics.py
"""

from State import State, Control
import numpy as np


class CarDynamics:
    """
    Implements the car dynamics and solver for the optimal control problem
    """

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

    def linearize(self, x_hat):
        """
        Returns A, B, c matrices for the system dynamics linearized
        around f(x_hat, 0)
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

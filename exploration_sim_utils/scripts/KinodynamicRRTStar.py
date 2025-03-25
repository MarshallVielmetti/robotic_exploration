# file: KindoynamicRRTStar.py

import numpy as np
from State import *
from CarDynamics import CarDynamics
from NumericalOptimalControl import NumericalOptimalControlSolver
from AnalyticalOptimalControl import AnalyticalOptimalControlSolver

import random

"""
Implements the algorithm described in the paper

Kinodynamic RRT*: Optimal Motion Planning for 
Systems with Linear Differential Constraints
"""


class KinodynamicRRTStar:
    def __init__(self, esdf: np.ndarray):

        # Euclidian signed distance field representing the environment
        self.esdf = esdf

        self.max_iter = 1000
        self.local_sampling_bias = 0.2

        self.r = 30.0  # The maximum cost threshold for a node to be considered "near"
        self.d_max = 5.0  # The maximum distance for a node to be considered "near"

        self.cost: dict[State, float] = {}
        self.parent: dict[State, State] = {}
        self.nodes: set[State] = set()

        self.car_dynamics = CarDynamics()
        # self.solver = NumericalOptimalControlSolver(self.car_dynamics)
        self.solver = AnalyticalOptimalControlSolver(self.car_dynamics)

        self.num_sampled_at_iter = []

    def reset(self) -> None:
        """
        Resets the graph
        """
        self.cost = {}
        self.parent = {}
        self.num_sampled_at_iter = []

    def plan(self, start: State, goal: State) -> list[State]:
        """
        Implements the algorithm described in Fig. 3 of the paper
        """
        self.reset()

        self.nodes.add(start)
        self.cost[start] = 0
        self.parent[start] = start
        self.cost[goal] = np.inf

        curr_iter = 0
        num_sampled = 0
        while curr_iter < self.max_iter:
            # Randomly sample an x_i in X_FREE
            x_i = self.sample_random_state()
            num_sampled += 1

            # If somehow that node is already in the graph, skip
            if x_i in self.nodes:
                continue

            # find nearest valid node in the graph
            x_nearest, nearest_cost = self.nearest_node(x_i)

            # if no valid nearest node, skip
            if x_nearest is None:
                continue

            print(f"Connection {x_nearest} -> {x_i}, cost {nearest_cost}")

            # This is a valid iteration, so increment the iteration count
            curr_iter += 1
            print(f"Sampled {num_sampled} nodes before making successful connection")
            self.num_sampled_at_iter.append(num_sampled)
            num_sampled = 0

            # Directly add x_i to the graph -- no "steer" function
            self.parent[x_i] = x_nearest
            self.cost[x_i] = self.cost[x_nearest] + self.cost_between(x_nearest, x_i)[0]

            # Find the set of nodes "near" to x_i that will be rewired
            candidates, cost_between = self.get_near_set(x_i, goal)

            # perform rewiring
            for x, x_i_to_x in zip(candidates, cost_between):
                self.cost[x] = self.cost[x_i] + x_i_to_x  # self.cost_between(x_i, x)[0]
                self.parent[x] = x_i

            # finally, add x_i to the list of nodes
            self.nodes.add(x_i)

            print(f"Current Iteration: {curr_iter}")

        # Find the optimal path
        return self.find_optimal_path(start, goal)

    def is_valid_state(self, x: State) -> bool:
        """
        Returns true if the state is valid (in the free set)
        """
        return (
            int(x.x) < self.esdf.shape[1]
            and int(x.y) < self.esdf.shape[0]
            # and self.esdf[int(x.y), int(x.x)] > 0
            and abs(x.theta) < np.pi
            and x.v < State.V_MAX
            and x.v > State.V_MIN
            and x.kappa < State.KAPPA_MAX
            and x.kappa > State.KAPPA_MIN
        )

    def is_valid_trajectory(self, trajectory: list[State]) -> bool:
        """
        Whether or not every state in the trajectory lies in x_free
        """
        return all(self.is_valid_state(x) for x in trajectory)

    # def is_valid_state(self, x: np.ndarray) -> bool:
    #     return self.is_valid_state(State(x[0], x[1], x[2], x[3], x[4]))

    def sample_random_state(self) -> State:
        """
        Samples a random x_i in X_free
        """
        # cand = get_random_state(0, self.esdf.shape[0], 0, self.esdf.shape[1])
        cand = self.weighted_random_sample()
        while not self.is_valid_state(cand):
            # cand = get_random_state(0, self.esdf.shape[0], 0, self.esdf.shape[1])
            cand = self.weighted_random_sample()

        return cand

    def weighted_random_sample(self) -> State:
        """
        Samples a random state near existing nodes in the graph
        """

        if random.random() < self.local_sampling_bias:
            # Sample near an existing node
            x_near = random.choice(list(self.nodes))

            # sample within radius corresponding to d_max
            sampling_radius = self.d_max * 0.9

            x_offset = random.uniform(-sampling_radius, sampling_radius)
            y_offset = random.uniform(-sampling_radius, sampling_radius)
            theta_offset = random.uniform(-np.pi / 4, np.pi / 4)
            v_offset = random.uniform(-1, 1)
            kappa_offset = random.uniform(-0.2, 0.2)

            cand = State(
                x_near.x + x_offset,
                x_near.y + y_offset,
                x_near.theta + theta_offset,
                x_near.v + v_offset,
                x_near.kappa + kappa_offset,
            )

            return cand

        else:
            return get_random_state(0, self.esdf.shape[0], 0, self.esdf.shape[1])

    def cost_between(
        self, x1: State, x2: State
    ) -> tuple[float, list[State], list[Control]]:
        """
        returns c*[x, x_i] as described in the paper
        """
        # Solve the optimal control problem
        return self.solver.solve(x1, x2, 10)

    def nearest_node(self, x_rand: State) -> tuple[State, float]:
        """
        Finds the nearest node in the tree to x_rand
        There is a maximum cost self.max_cost that a nearest node can have
        to ensure the linearized dynamics are valid. if no nodes in the graph
        are within the cost, then None is returned
        """
        x_near: State = None
        min_cost: float = np.inf

        # filter out nodes that are too far away
        candidates = [
            (x, np.linalg.norm(np.array([x.x, x.y]) - np.array([x_rand.x, x_rand.y])))
            for x in self.nodes
            if np.linalg.norm(np.array([x.x, x.y]) - np.array([x_rand.x, x_rand.y]))
            < self.d_max
        ]

        # sort by euclidian distance
        candidates.sort(key=lambda x: x[1])

        for x, _ in candidates[: min(10, len(candidates))]:
            # Make sure the node has a cost? not sure if this is necessary
            if x not in self.cost:
                continue

            # Check if the distance is within the maximum distance for the linearized dynamics to be valid
            dist = np.linalg.norm(np.array([x.x, x.y]) - np.array([x_rand.x, x_rand.y]))
            if dist > self.d_max:
                continue

            path_cost, state_trajectory, control_trajectory = self.cost_between(
                x, x_rand
            )

            if path_cost is None:
                continue

            # If the cost is too high, skip
            if path_cost >= self.r:
                print(f"Computed path Cost of {path_cost} too high")
                continue

            # check that the state trajecotory is collision free
            if not self.is_valid_trajectory(state_trajectory):
                continue

            # this is the value to be minimized
            cand_cost: float = self.cost[x] + path_cost

            if cand_cost < min_cost:
                x_near = x
                min_cost = cand_cost

        return x_near, min_cost

    def get_near_set(
        self, x_i: State, x_goal: State
    ) -> tuple[list[State], list[float]]:
        """
        Returns the set of nodes in the graph that are near x
        explicitly considering x_goal, and the cost to go from x_i to x
        All returned nodes should be rewired
        """

        X_CAND = []
        costs_between = []

        # filter out nodes that are too far away
        candidates = [
            (x, np.linalg.norm(np.array([x.x, x.y]) - np.array([x_i.x, x_i.y])))
            for x in self.nodes.union({x_goal})
            if np.linalg.norm(np.array([x.x, x.y]) - np.array([x_i.x, x_i.y]))
            < self.d_max
        ]

        # sort by euclidian distance
        candidates.sort(key=lambda x: x[1])

        for x, _ in candidates:
            path_cost, state_trajectory, _ = self.cost_between(x_i, x)

            # cost too high to be considered near
            if path_cost is None or path_cost >= self.r:
                continue

            cand_cost = self.cost[x_i] + path_cost

            # Cost is higher than what we have already so skip
            if x in self.cost and cand_cost > self.cost[x]:
                continue

            # check that the state trajectory is collision free
            for x_traj in state_trajectory:
                if not self.is_valid_state(x_traj):
                    continue

            # valid thing to rewire so add to X_CAND
            X_CAND.append(x)
            costs_between.append(path_cost)

        return X_CAND, costs_between

    def find_optimal_path(self, start: State, goal: State) -> list[State]:
        """
        Finds the optimal path from start to goal using the
        generated graph
        """
        path = [goal]
        control_path = []

        while path[-1] != start:
            path.append(self.parent[path[-1]])

        path.reverse()

        return path

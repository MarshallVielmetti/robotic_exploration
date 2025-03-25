# FILE: KinodynamicRRT.py


# Implementation of RRT* Algorithm for Kinodynamic Planning
from State import State, CarDynamics
import numpy as np


# CONSTANTS
V_MAX = 3  # max linear velocity
W_MAX = 1  # max angular velocity

A_MAX = 1  # max linear acceleration
ALPHA_MAX = 0.5  # max angular acceleration

GAMMA_RRT = 1.0
D_RRT = 2.0


class DictEntry:
    cost = np.inf
    edges = []
    parent = None


class KinodynamicRRT:
    def __init__(self, start, goal, esdf):
        self.start = start
        self.goal = goal
        self.esdf = esdf

        # self.nodes = [start]
        # self.edges = []

        self.graph: dict[State, DictEntry] = {}

        self.max_iter = 1000
        self.max_dist = 5

        # Weighted Euclidian Distance
        # theta is weighted more heavily than x and y, but v and w are weighted less
        self.R = np.diag([1, 1, 2.5, 0.5, 0.5])

        self.steer_dt = 10

        # Add the start node to the graph
        self.add_node(start)
        self.graph[start].cost = 0
        self.graph[start].parent = start

        self.car_dynamics = CarDynamics()

    def plan(self, iter=1000):

        for i in range(min(iter, self.max_iter)):
            # Sample a random state
            x_rand = self.sample_random_state()

            # Find the nearest node in the tree
            x_nearest = self.nearest_node(x_rand)

            tau_min, states, controlls = self.car_dynamics.steer(x_nearest, x_rand)

            # Find the new node
            # x_new = self.steer(x_nearest, x_rand)

            # Check if the new node is valid
            # if self.collision_free(x_nearest, x_new):
            if self.collision_free(states):
                self.add_to_tree(x_nearest, x_rand, tau_min)

    def add_to_tree(self, x_nearest, x_new, tau_min):
        # Find a set of near nodes
        X_near = self.near_set(x_new)

        # Add x_new to the tree
        self.add_node(x_new)

        x_min = x_nearest
        c_min = self.cost(x_nearest) + self.cost_between(x_nearest, x_new)

        # Connect along a minimum cost path
        for x_near in X_near:
            if (
                self.collision_free(x_near, x_new)
                and self.cost(x_near) + self.cost_between(x_near, x_new) < c_min
            ):
                x_min = x_near
                c_min = self.cost(x_near) + self.cost_between(x_near, x_new)

        # Add miminum cost edge to the tree
        self.add_edge(x_min, x_new)

        # Rewire the tree for all nearby nodes -- check if there is a lower cost path
        # to the tree through x_new
        for x_near in X_near:
            if self.collision_free(x_new, x_near) and self.cost(
                x_new
            ) + self.cost_between(x_new, x_near) < self.cost(x_near):

                # Rewire the tree
                self.rewire(x_new, x_near)

    def sample_random_state(self):
        x_rand = State()

        # Sample a random state
        x_rand.x = np.random.uniform(0, self.esdf.shape[0])
        x_rand.y = np.random.uniform(0, self.esdf.shape[1])
        x_rand.theta = np.random.uniform(-np.pi, np.pi)
        x_rand.v = np.random.uniform(0, V_MAX)
        x_rand.w = np.random.uniform(-W_MAX, W_MAX)

        return x_rand

    def nearest_node(self, x_rand):
        min_dist = np.inf
        min_node = None
        for node in self.graph.keys():
            dist = self.distance(node, x_rand)
            if dist < min_dist:
                min_dist = dist
                min_node = node

        return min_node

    def near_set(self, x_new):
        X_near = []

        distance_threshold = min(
            GAMMA_RRT * ((np.log(len(self.graph)) / len(self.graph)) ** (1 / D_RRT)),
            self.max_dist,
        )

        for node in self.graph.keys():
            if self.distance(node, x_new) < distance_threshold:
                X_near.append(node)

        return X_near

    def add_node(self, x_new):
        self.graph[x_new] = DictEntry()  # default values set in DictEntry

    def add_edge(self, x_from, x_to):
        self.graph[x_from].edges.append(x_to)
        self.graph[x_to].parent = x_from
        self.graph[x_to].cost = self.graph[x_from].cost + self.cost_between(
            x_from, x_to
        )

    def collision_free(self, x_from, x_to):
        # Check if the path between x_from and x_to is collision free
        x_i = int(x_to.x)
        y_i = int(x_to.y)

        if self.esdf[y_i, x_i] == np.inf:
            return False

        return True

    def cost(self, x):
        return self.graph[x].cost

    # assumes x1 has a cost stored in the graph
    def cost_between(self, x1, x2):
        """
        Fast cost estimation for RRT planning 
        """
        # Convert state objects to numpy arrays
        x1_np = np.array([x1.x, x1.y, x1.theta, x1.v, x1.kappa])
        x2_np = np.array([x2.x, x2.y, x2.theta, x2.v, x2.kappa])

        # use the fast estimator
        return self.car_dynamics.estimate_cost(x1_np, x2_np)

        # return self.distance(x1, x2)  # distance approxed as cost?

    def distance(self, x1, x2):
        # Weighted Euclidan Distance
        x = np.array([x1.x, x1.y, x1.theta, x1.v, x1.kappa]) - np.array(
            [x2.x, x2.y, x2.theta, x2.v, x2.kappa]
        )

        return np.sqrt(x.T @ self.R @ x)

    # compute optimal input to steer from x_nearest to x_rand
    # input is acceleration and angular acceleration
    def steer(self, x_from: State, x_to: State) -> State:
        dx = x_to.x - x_from.x
        dy = x_to.y - x_from.y

        # Compute the angle to the goal
        theta_des = np.arctan2(dy, dx)

        # normalize error to [-pi, pi]
        theta_error = (theta_des - x_from.theta + np.pi) % (2 * np.pi) - np.pi

        distance = np.sqrt(dx**2 + dy**2)
        a = 0.5 * (x_to.v - x_from.v)

        dv = x_to.v - x_from.v
        dw = x_to.kappa - x_from.kappa


        # # angular acceleration should be proportional to required change in angular velocity and angle
        # alpha = robot.kp_alpha * theta_error + robot.kd_alpha * dw

        # Limit the acceleration to be in bounds
        a = np.clip(a, -A_MAX, A_MAX)
        alpha = np.clip(alpha, -ALPHA_MAX, ALPHA_MAX)

        # determine new state
        x_new = State()
        x_new.x = x_from.x + x_from.v * np.cos(x_from.theta) * self.steer_dt
        x_new.y = x_from.y + x_from.v * np.sin(x_from.theta) * self.steer_dt
        x_new.theta = x_from.theta + x_from.w * self.steer_dt

        # normalize theta
        x_new.theta = (x_new.theta + np.pi) % (2 * np.pi) - np.pi

        x_new.v = x_from.v + a * self.steer_dt
        x_new.w = x_from.w + alpha * self.steer_dt

        return x_new

    # update the graph so that x_to's parent is x_from
    def rewire(self, x_from, x_to):
        parent = self.graph[x_to].parent
        self.graph[parent].edges.remove(x_to)
        self.graph[x_to].parent = x_from
        self.graph[x_to].cost = self.graph[x_from].cost + self.cost_between(
            x_from, x_to
        )
        self.graph[x_from].edges.append(x_to)

    def plot(self, axs):
        for node in self.graph.keys():
            axs.plot(node.x, node.y, "ro")
            for edge in self.graph[node].edges:
                axs.plot(
                    [node.x, edge.x],
                    [node.y, edge.y],
                    color="black",
                    linestyle="-",
                    linewidth=0.5,
                )



    # def c_star(self, x_from, x_to):

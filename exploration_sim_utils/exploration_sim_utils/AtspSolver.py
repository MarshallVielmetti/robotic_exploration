import rclpy
from rclpy.node import Node
import numpy as np

from exploration_sim_msgs.msg import TspProblem
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import lkh


class AtspSolver(Node):
    def __init__(self):
        super().__init__("atsp_solver")
        self.get_logger().info("Initializing ATSP Solver node")

        self.problem = self.init_problem()

        self.tsp_msg_sub_ = self.create_subscription(
            TspProblem, "tsp_problem", self.tsp_callback, 10
        )

        self.path_pub_ = self.create_publisher(Path, "coverage_path", 10)

        self.tsp_msg_sub_

    def tsp_callback(self, msg: TspProblem):
        self.get_logger().debug(f"Received TSP problem of length {len(msg.nodes)}")

        if len(msg.nodes) < 3:
            self.get_logger().debug("Not enough nodes to solve TSP")
            self.handle_degenerate_case(msg)
            return

        # Update and solve problem
        self.update_problem(msg)
        output = lkh.solve(problem=self.problem, max_trials=1000, runs=10)

        # Permute the nodes in the message to match the optimal tour
        optimal_tour = output[0]

        # Find the index of robot position (last node in message) in the optimal tour
        start_idx = optimal_tour.index(len(msg.nodes))
        optimal_tour = optimal_tour[start_idx:] + optimal_tour[:start_idx]

        # Reorder the nodes to reflect the optimal tour
        reordered_nodes = [msg.nodes[i - 1] for i in optimal_tour]

        # publish the reordered nodes as a nav_msgs/Path
        self.path_pub_.publish(self.create_path_msg(msg, reordered_nodes))

    def init_problem(self):
        problem = lkh.LKHProblem()

        problem.type = "ATSP"

        # Set to 0 to indicate that the dimension is not known yet
        problem.dimension = 0

        problem.edge_weight_type = "EXPLICIT"
        problem.edge_weight_format = "FULL_MATRIX"
        problem.edge_weights = []

        problem.TRACE_LEVEL = 1  # Amount of output (0=silent, 1=normal, 2=verbose)
        problem.RUNS = 10  # Number of independent runs
        problem.CANDIDATE_SET_TYPE = "ALPHA"  # Type of candidate set

        return problem

    def update_problem(self, msg: TspProblem):
        self.problem.dimension = len(msg.nodes)

        weights = np.array(msg.weights, dtype=np.float32).reshape(
            (self.problem.dimension, self.problem.dimension)
        )

        scale_factor = 100
        int_weights = np.round(weights * scale_factor).astype(np.int32)
        int_weights[np.isinf(weights)] = 999999

        self.problem.edge_weights = int_weights  # int_weights.flatten().tolist()

    def create_path_msg(self, msg, nodes) -> Path:
        path_msg = Path()
        path_msg.header.stamp = msg.header.stamp
        path_msg.header.frame_id = msg.header.frame_id

        for node in nodes:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = node.position.x
            pose.pose.position.y = node.position.y
            path_msg.poses.append(pose)

        return path_msg

    def handle_degenerate_case(self, msg: TspProblem):
        if len(msg.nodes) < 2:
            self.get_logger().info("Not enough nodes to solve TSP")
            return

        # There are exactly 2 nodes

        path_msg = Path()
        path_msg.header = msg.header

        robot_pose = PoseStamped()
        robot_pose.header = msg.header
        robot_pose.pose.position = msg.nodes[1].position

        other_pose = PoseStamped()
        other_pose.header = msg.header
        other_pose.pose.position = msg.nodes[0].position

        path_msg.poses = [robot_pose, other_pose]

        self.path_pub_.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    atsp_solver = AtspSolver()
    rclpy.spin(atsp_solver)

    atsp_solver.destroy_node()
    rclpy.shutdown()

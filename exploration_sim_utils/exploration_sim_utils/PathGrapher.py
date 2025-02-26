import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

from ament_index_python.packages import get_package_share_directory

import matplotlib.pyplot as plt
from datetime import datetime
import os

class PathGrapher(Node):
    def __init__(self):
        super().__init__('path_grapher')
        self.subscriber_ = self.create_subscription(Path, 'path', self.path_callback, 10)
        self.subscriber_


    def path_callback(self, msg: Path):
        self.get_logger().info("Received path message")
        path_vector = msg._poses

        xs = []
        ys = []

        for vec in path_vector:
            xs.append(vec.pose.position.x) 
            ys.append(vec.pose.position.y)
        
        plt.plot(xs, ys)

        img_save_dir = os.path.join(get_package_share_directory('exploration_sim_utils'), 'paths', f"path_{datetime.now()}.png")
        plt.title(f"Path from ({xs[0]:.2f}, {ys[0]:.2f}) to ({xs[-1]:.2f}, {ys[-1]:.2f})")
        plt.gca().set_aspect('equal')
        plt.gca().set_adjustable('datalim')
        plt.gca().set_xlim([min(xs) - 1, max(xs) + 1])
        plt.grid()
        plt.savefig(img_save_dir)
        plt.clf()


def main(args=None):
    rclpy.init(args=args)

    path_grapher = PathGrapher()
    
    rclpy.spin(path_grapher)

    path_grapher.destroy_node()
    rclpy.shutdown()




import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
import os
from datetime import datetime

import matplotlib
matplotlib.use('agg')

class FitSplines(Node):
  def __init__(self):
    super().__init__('fit_splines')
    self.get_logger().info("Initializing FitSplines node")

    self.path_subcriber_ = self.create_subscription(Path, 'path', self.path_callback, 10)
    self.path_subcriber_

    self.path_publisher_ = self.create_publisher(Path, 'spline_path', 10)


  def path_callback(self, msg: Path):
    self.get_logger().info("Received path message")

    path_vector = msg._poses

    t = np.linspace(0, 1, len(path_vector))
    xs = [vec.pose.position.x for vec in path_vector]
    ys = [vec.pose.position.y for vec in path_vector]

    spl_x = interpolate.splrep(t, xs, s=3, k=3)
    spl_y = interpolate.splrep(t, ys, s=3, k=3)

    tt = np.linspace(0, 1, 1000)
    xx = interpolate.splev(tt, spl_x)
    yy = interpolate.splev(tt, spl_y)

    plt.plot(xs, ys, 'bo', xx, yy, 'r')


    plt.gca().set_aspect('equal')
    plt.gca().set_adjustable('datalim')
    plt.grid()

    img_save_dir = os.path.join(get_package_share_directory('exploration_sim_utils'), 'paths', f"spline_{datetime.now()}.png")
    plt.savefig(img_save_dir)

    plt.clf()

    spline_path = Path()
    spline_path.header = msg.header

    for i in range(len(xx)):
      pose = PoseStamped()
      pose.header = msg.header
      pose.pose.position.x = xx[i]
      pose.pose.position.y = yy[i]
      spline_path.poses.append(pose)

    self.path_publisher_.publish(spline_path)



def main(args=None):
  rclpy.init(args=args)

  fit_splines = FitSplines()

  rclpy.spin(fit_splines)

  fit_splines.destroy_node()
  rclpy.shutdown()
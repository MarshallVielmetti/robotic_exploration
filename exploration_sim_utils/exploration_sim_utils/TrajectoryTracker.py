import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import numpy as np




class TrajectoryTracker(Node):
  def __init__(self):
    super().__init__('trajectory_tracker')

    self.get_logger().info("Initializing TrajectoryTracker node")

    self.path_subscriber_ = self.create_subscription(Path, 'spline_path', self.path_callback, 10)
    self.path_subscriber_

    self.pose_subscriber_ = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
    self.pose_subscriber_

    self.control_publisher_ = self.create_publisher(PoseStamped, 'control', 10)
    self.control_update_timer = self.create_timer(0.1, self.control_update)

    self.path = None
    self.current_pose = None

  def path_callback(self, msg: Path):
    self.get_logger().info("Received path message")
    self.path = msg
  
  def pose_callback(self, msg: PoseStamped):
    self.get_logger().info("Received pose message")
    self.current_pose = msg
  
  def control_update(self):
    if self.path is None or self.current_pose is None:
      return

    # Find the closest point on the path to the current pose


  
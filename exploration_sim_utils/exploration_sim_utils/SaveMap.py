
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy


import numpy as np

from ament_index_python import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool

import os
from datetime import datetime

class SaveMap(Node):
  def __init__(self):
    super().__init__('save_map_script')

    qos = QoSProfile(
      history=QoSHistoryPolicy.KEEP_LAST,
      depth=1,
      reliability=QoSReliabilityPolicy.RELIABLE
    )

    self.subscriber_ = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)

    self.message_processed_ = False

  def map_callback(self, msg: OccupancyGrid):
    self.get_logger().info("Received map message!")

    if (self.message_processed_):
      return;

    self.message_processed_ = True

    self.destroy_subscription(self.subscriber_)
    self.subscriber_ = None

    # load the map into a numpy array
    map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)

    # ensure the directory exists, if not create it
    maps_dir = os.path.join(get_package_share_directory('exploration_sim_utils'), 'maps')
    os.makedirs(maps_dir, exist_ok=True)

    # create save file name
    timestamp = datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    save_dir = os.path.join(maps_dir, f"map_{timestamp}.csv")

    # save as a csv file
    np.savetxt(save_dir, map_data, delimiter=',', fmt='%d')

    # Shutdown the node after callback exits
    self.get_logger().info(f"Map saved to {save_dir}. Shutting down ...")
    self.create_timer(0.5, self.shutdown)

  def shutdown(self):
    rclpy.shutdown()


def main(args=None):
  rclpy.init(args=args)

  save_map = SaveMap()

  try:
    rclpy.spin(save_map)
  finally:
    save_map.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()

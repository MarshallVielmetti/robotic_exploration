import rclpy
from rclpy.node import Node

from exploration_sim_msgs.msg import ConnectivityGraph
from nav_msgs.msg import OccupancyGrid

from ament_index_python.packages import get_package_share_directory

import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime



class ConnectivityGraphPlotter(Node):
  def __init__(self):
    super().__init__('connectivity_graph_plotter')

    self.get_logger().info("Initializing ConnectivityGraphPlotter node")

    self.graph_subscriber_ = self.create_subscription(ConnectivityGraph, 'connectivity_graph', self.graph_callback, 10)
    self.graph_subscriber_

    self.zone_subscriber_ = self.create_subscription(OccupancyGrid, 'zones', self.zone_callback, 10)
    self.zone_subscriber_

    self.zone_ = None
    self.graph_ = None

  def zone_callback(self, msg: OccupancyGrid):
    self.get_logger().info("Received zone message")
    self.zone_ = msg

  def graph_callback(self, msg: ConnectivityGraph):
    self.get_logger().info("Received graph message")
    self.graph_ = msg

    self.plot_graph()
  
  def plot_graph(self):
    if self.zone_ is None or self.graph_ is None:
      return

    # Plot the zones
    # plt.imshow(self.zone_.data, cmap='gray')

    # Plot the vertices
    for vertex in self.graph_.nodes:
      plt.plot(vertex.x, vertex.y, 'ro')

    # plot the edges between the vertices
    # the edges are passed as a 1D array, but is really a 2D connectivity array
    num_rows = self.graph_.rows

    # transform in a 2D numpy array
    # self.graph_.edges is an array of Edges, which contain a float32 (cost), and a uint8 (label)
    # connectivity = np.array(self.graph_.edges, dtype=np.uint8).reshape(num_rows, -1)
    connectivity = np.array([edge.label for edge in self.graph_.edges], dtype=np.uint8).reshape(num_rows, -1)

    # iterate over the connectivity array -- it is symmetric
    for i in range(num_rows):
      for j in range(i, num_rows):
        if connectivity[i][j] == 0:
          continue

        # plot line between the vertices i and j
        plt.plot([self.graph_.nodes[i].x, self.graph_.nodes[j].x], [self.graph_.nodes[i].y, self.graph_.nodes[j].y], 'b')


    img_save_dir = os.path.join(get_package_share_directory('exploration_sim_utils'), 'paths', f"graph_{datetime.now()}.png")
    plt.title("Connectivity Graph")
    plt.savefig(img_save_dir)
    plt.clf()

    self.get_logger().info(f"Saved connectivity graph to {img_save_dir}")
  

def main(args=None):
  rclpy.init(args=args)

  cgp = ConnectivityGraphPlotter()

  rclpy.spin(cgp)

  cgp.destroy_node()
  rclpy.shutdown()



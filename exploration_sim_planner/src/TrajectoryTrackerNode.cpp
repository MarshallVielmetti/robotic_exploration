#include <torch/torch.h>
#include <rclcpp/rclcpp.hpp>


class TrajectoryTrackerNode : public rclcpp::Node {
public:
  TrajectoryTrackerNode() : Node("trajectory_tracker") {
    RCLCPP_INFO(get_logger(), "Trajectory Tracker Node has been started.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
#include "exploration_sim_planner/nodes/PoseTransformListenerNode.hpp"

#include <rclcpp/rclcpp.hpp>

PoseTransformListenerNode::PoseTransformListenerNode()
    : Node("pose_transform_listener") {
  RCLCPP_INFO(this->get_logger(), "Starting Pose Transform Listener");

  // Create a publisher for the pose
  pose_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", 10);

  // Create a transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create a timer to trigger the pose publishing
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PoseTransformListenerNode::on_timer, this));
}

void PoseTransformListenerNode::on_timer() {
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform =
        tf_buffer_->lookupTransform("map", "diff_drive", tf2::TimePointZero);
  } catch (const tf2::TransformException &e) {
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", e.what());
    return;
  }

  geometry_msgs::msg::PoseStamped pose;

  pose.header.stamp = this->now();
  pose.header.frame_id = "map";
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;

  pose_publisher_->publish(pose);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseTransformListenerNode>());
  rclcpp::shutdown();
  return 0;
}
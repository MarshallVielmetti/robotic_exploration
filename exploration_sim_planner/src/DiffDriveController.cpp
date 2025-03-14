#include "exploration_sim_planner/DiffDriveController.hpp"

using namespace std::chrono_literals;

const std::chrono::milliseconds DiffDriveController::CONTROL_FREQUENCY = 100ms;
const double DiffDriveController::CONTROL_FREQUENCY_SEC = 0.1;

DiffDriveController::DiffDriveController() : Node("diff_drive_controller") {
  RCLCPP_INFO(this->get_logger(), "DiffDriveController node started");

  // Create a publisher for the control commands
  cmd_vel_publisher_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Create a subscriber for the control commands
  control_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "control_input", 10,
      std::bind(&DiffDriveController::control_callback, this,
                std::placeholders::_1));

  // Create a timer to publish control commands at a fixed frequency
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(DiffDriveController::CONTROL_FREQUENCY),
      std::bind(&DiffDriveController::publish_velocity_setpoint, this));

  // Initialize the velocity setpoint
  velocity_setpoint_ = Eigen::Vector2d::Zero();
  control_value_ = Eigen::Vector2d::Zero();
}

void DiffDriveController::control_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  control_value_ << msg->linear.x, msg->angular.z;
}

void DiffDriveController::publish_velocity_setpoint() {
  // Update the velocity setpoint using the control value
  velocity_setpoint_ += control_value_ * CONTROL_FREQUENCY_SEC;

  // Publish the velocity setpoint
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = velocity_setpoint_.x();
  cmd_vel.angular.z = velocity_setpoint_.y();
  cmd_vel_publisher_->publish(cmd_vel);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}
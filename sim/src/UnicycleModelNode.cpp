#include "sim/UnicycleModelNode.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <iostream>

using namespace std::chrono_literals;

UnicycleModelNode::UnicycleModelNode(UnicycleModel &model)
    : Node("unicycle_model_node"), model_(model) {
  // Create a timer that fires at the frequency of the control update
  control_timer_ = this->create_wall_timer(
      16ms, std::bind(&UnicycleModelNode::control_timer_callback, this));

  // Initialize the subscriber
  this->control_input_sub_ =
      this->create_subscription<geometry_msgs::msg::Twist>(
          "control_input", 10,
          std::bind(&UnicycleModelNode::control_input_callback, this,
                    std::placeholders::_1));

  // Initialize the publisher
  this->command_velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("command_velocity", 10);
}

void UnicycleModelNode::control_input_callback(
    const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  // Store the control input in the model
  model_.set_acceleration(msg->linear.x, msg->angular.z);
}

void UnicycleModelNode::control_timer_callback() {

  // Update the model
  model_.update(0.016);

  // publish the commanded velocity

  auto msg = geometry_msgs::msg::Twist();

  auto linear = model_.get_linear_velocity();
  auto angular = model_.get_angular_velocity();

  msg.linear.x = linear(0);
  msg.linear.y = linear(1);
  msg.linear.z = linear(2);

  msg.angular.x = angular(0);
  msg.angular.y = angular(1);
  msg.angular.z = angular(2);

  command_velocity_pub_->publish(msg);
}

int main(int args, char *argv[]) {

  std::cout << "Hello, World!" << std::endl;

  return 1;
}
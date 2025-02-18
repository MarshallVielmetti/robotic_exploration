#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "UnicycleModel.hpp"

class UnicycleModelNode : public rclcpp::Node {
    public:
        UnicycleModelNode(UnicycleModel &model);

    private:
        // Fires at the frequency of the control update
        void control_timer_callback();

        // Callback for when a new control input is provided
        // stores the control input in the model
        void control_input_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);


    // Variables
    UnicycleModel &model_;

    // Timer for control update
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Subscriber for control input
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_input_sub_;

    // Publisher for command velocity
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_velocity_pub_;

};
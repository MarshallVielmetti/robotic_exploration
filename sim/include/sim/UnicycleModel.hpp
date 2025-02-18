#pragma once

#include <Eigen/Dense>
#include <algorithm>

/*
 * Unicycle model for a robot.
 */
class UnicycleModel {
public:
  UnicycleModel(double wheel_base, double max_steering_angle, double vmax,
                double wmax, double v_dot_max, double w_dot_max)
      : wheel_base_(wheel_base), max_steering_angle_(max_steering_angle),
        vmax_(vmax), v_dot_max_(v_dot_max), wmax_(wmax), w_dot_max_(w_dot_max) {
    linear_vel_ = 0.0;
    angular_vel_ = 0.0;
    linear_acc_ = 0.0;
    angular_acc_ = 0.0;

    vel_linear_ = Eigen::Vector3f::Zero();
    vel_angular_ = Eigen::Vector3f::Zero();
  }

  // Update current acceleration / angular acceleration and gate values
  void set_acceleration(double linear_acc, double angular_acc) {
    linear_acc_ = std::max(-v_dot_max_, std::min(v_dot_max_, linear_acc));
    angular_acc_ = std::max(-w_dot_max_, std::min(w_dot_max_, angular_acc));
  }

  void update(double dt) {
    // Update the velocity based on the acceleration
    linear_vel_ += linear_acc_ * dt;
    angular_vel_ += angular_acc_ * dt;

    // Clamp the velocity
    linear_vel_ = std::max(0.0, std::min(vmax_, linear_vel_));
    angular_vel_ = std::max(-wmax_, std::min(wmax_, angular_vel_));
  }

  Eigen::Vector3f get_linear_velocity() const { return vel_linear_; }

  Eigen::Vector3f get_angular_velocity() const { return vel_angular_; }

private:
  // store current command_velocity (in robot frame)
  double linear_vel_, angular_vel_;
  double linear_acc_, angular_acc_;

  double wheel_base_;
  double max_steering_angle_;

  double vmax_, v_dot_max_;
  double wmax_, w_dot_max_;

  Eigen::Vector3f vel_linear_;
  Eigen::Vector3f vel_angular_;
};
/*
 * File: pid_controller.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 16th September 2019 7:08:26 pm
 */

#include "controller/pid_controller.h"

using namespace minjun;
using namespace minjun::utils;

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd) {
  LOG_INFO("Init PID Controller with param kp = %.3f, ki = %.3f, kd = %.3f",
           kp_, ki_, kd_);
}

ControlCommand PIDController::Control(
    const utils::Odom& odom,
    const std::vector<planner::PlannerPoint>& planner_points) {
  ControlCommand control_command;
  if (planner_points.size() < 3) {
    LOG_INFO("Reach destination");
    control_command.brake = 10.0;
    return control_command;
  }
  auto current_pos = odom.GetPosition();
  auto current_speed = odom.GetSpeed();
  auto current_yaw = odom.GetYaw() / 180.0 * M_PI;

  auto look_ahead_point = planner_points[3];
  auto target_pos = look_ahead_point.GetPoint();
  auto target_speed = look_ahead_point.GetSpeed();
  auto target_yaw = look_ahead_point.GetYaw();

  // PID on throttle
  double speed_error = target_speed - current_speed;
  double speed_error_derivative = speed_error / delta_t_;
  speed_error_integral_ += speed_error * delta_t_;

  double throttle_feedback = kp_ * speed_error + ki_ * speed_error_integral_ +
                             kd_ * speed_error_derivative;
  
  double throttle_feedforward = 0.0;
  if (target_speed <= 6.0) {
    throttle_feedforward = 0.15 + target_speed / 6.0 * (0.6 - 0.15);
  } else if (target_speed <= 11.5) {
    throttle_feedforward = 0.6 + (target_speed - 6.0) / (11.5- 6.0) * (0.8 - 0.6);
  } else {
    throttle_feedforward = 0.8 + (target_speed - 11.5) / 85.0;
  }

  control_command.throttle = std::min(std::max(throttle_feedback + throttle_feedforward, 0.0), 1.0);

  // PID on steer
  double steer_limit = 15.0 / 180.0 * M_PI;
  target_yaw = std::atan2(target_pos.y_ - current_pos.y_, target_pos.x_ - current_pos.x_);
  double yaw_error = target_yaw - current_yaw;
  if (yaw_error <= -M_PI) {
    yaw_error += 2 * M_PI;
  } else if (yaw_error >= M_PI) {
    yaw_error -= 2 * M_PI;
  }

  double cross_track_error = std::cos(current_yaw) * (target_pos.x_ - current_pos.x_) + 
                             std::sin(current_yaw) * (target_pos.y_ - current_pos.y_);
  double cross_track_angle = std::atan(5.0 * cross_track_error / (5.0 + current_speed));

  cross_track_angle = 0.0;
  std::cout << "target speed: " << target_speed << std::endl;
  std::cout << "current speed: " << current_speed << std::endl;
  control_command.steer = std::min(std::max(yaw_error + cross_track_angle, -steer_limit), steer_limit);
  return control_command;
}



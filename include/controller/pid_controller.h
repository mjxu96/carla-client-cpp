/*
 * File: pid_controller.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 16th September 2019 7:05:43 pm
 */

#ifndef MINJUN_CONTROL_PID_CONTROLLER_H_
#define MINJUN_CONTROL_PID_CONTROLLER_H_

#include "controller/base_controller.h"
#include <cmath>

namespace minjun {

class PIDController : public Controller {
 public:
  PIDController(double kp, double ki, double kd);
  ControlCommand Control(
      const utils::Odom& odom,
      const std::vector<planner::PlannerPoint>& planner_points) override;

 private:
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;

  // TODO add this delta t in configuration file
  double delta_t_ = 1.0 / 30.0;
  double speed_error_integral_ = 0.0;
};
}  // namespace minjun

#endif
/*
 * File: base_controller.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Sunday, 15th September 2019 3:12:39 pm
 */

#ifndef MINJUN_BASE_CONTROLLER_H_
#define MINJUN_BASE_CONTROLLER_H_

#include "controller/common.h"
#include "planner/planner.h"
#include "utils/macrologger.h"

namespace minjun {

class Controller {
 public:
  // virtual ControlCommand Control(const VehicleState& vehicle_state) = 0;
  virtual ControlCommand Control(
      const utils::Odom& odom,
      const std::vector<planner::PlannerPoint>& planner_points) = 0;
};

}  // namespace minjun

#endif
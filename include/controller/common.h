/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 16th September 2019 7:03:20 pm
 */

#ifndef MINJUN_CONTROL_COMMON_H_
#define MINJUN_CONTROL_COMMON_H_

#include "utils/common.h"

namespace minjun {
struct ControlCommand {
  double throttle = 0.0;
  double steer = 0.0;
  double brake = 0.0;
};

struct VehicleState {
  utils::Point3d position;
  double yaw = 0.0;
};

}

#endif
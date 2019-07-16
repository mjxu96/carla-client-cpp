/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 15th July 2019 9:30:33 pm
 */

#ifndef MINJUN_PLANNER_COMMON_H_
#define MINJUN_PLANNER_COMMON_H_

#include "router/common.h"

namespace minjun {
namespace planner {

class PlannerPoint {
public:
  PlannerPoint(Point3d point, double speed, double yaw);
  Point3d GetPoint() const;
  double GetSpeed() const;
  double GetYaw() const;
private:
  Point3d point_;
  double speed_;
  double yaw_;
};

class Utils {

};
  
} // namespace planner
} // namespace minjun


#endif
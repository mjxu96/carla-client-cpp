/*
 * File: planner.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 15th July 2019 8:52:38 pm
 */

#ifndef MINJUN_PLANNER_H_
#define MINJUN_PLANNER_H_

#include "router/router.h"
#include "planner/common.h"

namespace minjun {

class Planner {
public:
  Planner(Point3d start_point, Point3d end_point);
  std::vector<planner::PlannerPoint> GetPlannerPoints();
private:
  Point3d start_point_;
  Point3d end_point_;
};

} // namespace minjun

#endif
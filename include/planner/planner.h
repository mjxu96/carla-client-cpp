/*
 * File: planner.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 15th July 2019 8:52:38 pm
 */

#ifndef MINJUN_PLANNER_H_
#define MINJUN_PLANNER_H_

#include "planner/common.h"
#include "router/router.h"
#include "utils/constants.h"
#include "utils/macrologger.h"

namespace minjun {

class Planner {
 public:
  Planner(Point3d start_point, Point3d end_point,
          boost::shared_ptr<carla::client::Map> map_ptr, double init_yaw = 0.0,
          double init_speed = 0.0);
  std::vector<planner::PlannerPoint> GetPlannerPoints();

 private:
  Point3d start_point_;
  Point3d end_point_;
  double init_yaw_{0.0};  // degree
  double init_speed_{0.0};
  std::vector<Point3d> router_points_;
  boost::shared_ptr<carla::client::Map> map_ptr_{nullptr};
};

}  // namespace minjun

#endif
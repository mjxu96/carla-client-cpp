/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 15th July 2019 9:30:33 pm
 */

#ifndef MINJUN_PLANNER_COMMON_H_
#define MINJUN_PLANNER_COMMON_H_

#include "utils/common.h"

#include "router/common.h"
#include "utils/macrologger.h"

namespace minjun {
namespace planner {

class PlannerPoint {
 public:
  PlannerPoint(utils::Point3d point, double speed, double yaw);
  utils::Point3d GetPoint() const;
  double GetSpeed() const;
  double GetYaw() const;

 private:
  utils::Point3d point_;
  double speed_;
  double yaw_;
};

class Utils {
 public:
  // In degree
  static double GetYaw(const utils::Point3d& p1, const utils::Point3d& p2);
  // static double GetDistance(const Point3d& p1, const Point3d& p2);
  // Solve function like a*x^2 + b*x + c = 0;
  static boost::optional<std::pair<double, double>> SolveFunction(
      const double a, const double b, const double c);
  // Get yaw after rotate (-180.0 - 180.0)
  static double AfterRotate(double init_yaw, double target_yaw, double t,
                            const double yaw_rate_limit);
};

}  // namespace planner
}  // namespace minjun

#endif
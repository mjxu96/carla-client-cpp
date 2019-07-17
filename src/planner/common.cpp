/*
 * File: common.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 15th July 2019 9:33:33 pm
 */

#include "planner/common.h"

using namespace minjun;
using namespace minjun::utils;
using namespace minjun::planner;

PlannerPoint::PlannerPoint(Point3d point, double speed, double yaw)
    : point_(std::move(point)), speed_(speed), yaw_(yaw) {}

Point3d PlannerPoint::GetPoint() const { return point_; }

double PlannerPoint::GetSpeed() const { return speed_; }

double PlannerPoint::GetYaw() const { return yaw_; }

double Utils::GetYaw(const Point3d& p1, const Point3d& p2) {
  double yaw = std::atan2(p2.y_ - p1.y_, p2.x_ - p1.x_);
  return yaw / M_PI * 180.0;
}

boost::optional<std::pair<double, double>> Utils::SolveFunction(
    const double a, const double b, const double c) {
  if (std::abs(a) <= 0.000001) {
    if (std::abs(b) <= 0.00001) {
      return boost::none;
    } else {
      return std::make_pair(-c / b, -c / b);
    }
  }
  if (b * b - 4 * a * c < 0) {
    return boost::none;
  }
  double delta = std::sqrt(b * b - 4 * a * c);
  double x1 = (-b - delta) / (2 * a);
  double x2 = (-b + delta) / (2 * a);
  return std::make_pair(x1, x2);
}

double Utils::AfterRotate(double init_yaw, double target_yaw, double t,
                          const double yaw_rate_limit) {
  double res_yaw = init_yaw;
  if (init_yaw >= 0 && target_yaw >= init_yaw) {
    res_yaw = (((target_yaw - init_yaw) / yaw_rate_limit <= t)
                   ? target_yaw
                   : (init_yaw + t * yaw_rate_limit));
  } else if (init_yaw >= 0 && target_yaw < init_yaw) {
    if (target_yaw < init_yaw - 180.0) {
      target_yaw += 360.0;
      res_yaw = (((target_yaw - init_yaw) / yaw_rate_limit <= t)
                     ? target_yaw
                     : (init_yaw + t * yaw_rate_limit));
    } else {
      res_yaw = (((target_yaw - init_yaw) / (-yaw_rate_limit) <= t)
                     ? target_yaw
                     : (init_yaw - t * yaw_rate_limit));
    }
  } else if (init_yaw < 0 && target_yaw <= init_yaw) {
    res_yaw = (((target_yaw - init_yaw) / (-yaw_rate_limit) <= t)
                   ? target_yaw
                   : (init_yaw - t * yaw_rate_limit));
  } else if (init_yaw < 0 && target_yaw > init_yaw) {
    if (target_yaw > init_yaw + 180.0) {
      target_yaw -= 360.0;
      res_yaw = (((target_yaw - init_yaw) / (-yaw_rate_limit) <= t)
                     ? target_yaw
                     : (init_yaw - t * yaw_rate_limit));
    } else {
      res_yaw = (((target_yaw - init_yaw) / yaw_rate_limit <= t)
                     ? target_yaw
                     : (init_yaw + t * yaw_rate_limit));
    }
  } else {
    LOG_ERROR("You must kidding me");
  }

  while (res_yaw > 180.0) {
    res_yaw -= 360.0;
  }
  while (res_yaw <= -180.0) {
    res_yaw += 360.0;
  }
  return res_yaw;
}
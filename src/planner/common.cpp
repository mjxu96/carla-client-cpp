/*
 * File: common.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 15th July 2019 9:33:33 pm
 */

#include "planner/common.h"

using namespace minjun;
using namespace minjun::planner;

PlannerPoint::PlannerPoint(Point3d point, double speed, double yaw) :
  point_(std::move(point)), speed_(speed), yaw_(yaw) {}

Point3d PlannerPoint::GetPoint() const {return point_;}

double PlannerPoint::GetSpeed() const {return speed_;}

double PlannerPoint::GetYaw() const {return yaw_;}

/*
 * File: common.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 16th July 2019 9:08:17 pm
 */

#include "utils/common.h"


using namespace minjun::utils;
using namespace carla::geom;
using namespace carla::client;

// Point3d
bool Point3d::operator==(const Point3d& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0)) < 0.001;
}

double Point3d::Distance(const Point3d& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0));
}

std::string Point3d::ToString() const {
  return std::string("[") + std::to_string(x_) + std::string(", ") +
         std::to_string(y_) + std::string(", ") + std::to_string(z_) +
         std::string("]");
}


// Line2d
Line2d::Line2d(const Point3d& p1, const Point3d& p2) :
  start_(p1), end_(p2) {}

double Line2d::DotMul(const Line2d& l) {
  double v1_x = end_.x_ - start_.x_;
  double v1_y = end_.y_ - start_.y_;
  double v2_x = l.end_.x_ - l.start_.x_;
  double v2_y = l.end_.y_ - l.start_.y_;
  return v1_x * v2_x + v1_y * v2_y;
}

double Line2d::Distance(const Point3d& p, bool is_projection) {
  using point_type = boost::geometry::model::d2::point_xy<double>;
  using linestring_type = boost::geometry::model::linestring<point_type>;
  point_type pb(p.x_, p.y_);
  linestring_type line;
  line.push_back(point_type(start_.x_, start_.y_));
  line.push_back(point_type(end_.x_, end_.y_));
  if (is_projection) {
    return boost::geometry::distance(line, pb);
  }
  return -1.0;
}

// Map utils
MapUtils::MapUtils(boost::shared_ptr<carla::client::Map> map_ptr) :
  map_ptr_(std::move(map_ptr)) {}

bool MapUtils::IsInJunction(const Point3d& p) {
  Location location(p.x_, p.y_, p.z_);
  auto waypoint = map_ptr_->GetWaypoint(location);
  return waypoint->IsJunction();
}

size_t MapUtils::FindBehindIndex(const std::vector<Point3d>& points, const Point3d& current_point) {
  if (points.size() <= 1) {
    return 0;
  }
  double min_distance = std::numeric_limits<double>::max();
  double alter_min_distance = std::numeric_limits<double>::max();
  bool is_find = false;
  size_t index = 0;
  size_t alter_index = 0;
  for (size_t i = 0; i < points.size() - 1; i++) {
    Line2d line(points[i], points[i+1]);
    double distance = line.Distance(current_point);
    // if (!IsWithinSegment(points[i], points[i+1], current_point)) {
    //   if (distance < alter_min_distance) {
    //     alter_index = i;
    //     alter_min_distance = distance;
    //   }
    //   continue;
    // }
    if (distance < min_distance) {
      is_find = true;
      min_distance = distance;
      index = i;
    }
  }
  if (!is_find) {
    std::cout << "not found!!!!!!!!!!!!!! use alter: " << alter_index << std::endl;
    return alter_index;
  }
  return index;
}

bool MapUtils::IsWithinSegment(const Point3d& p1, const Point3d& p2, const Point3d& p) {
  Line2d out_line1(p1, p);
  Line2d out_line2(p2, p);
  Line2d in_line1(p1, p2);
  Line2d in_line2(p2, p1);
  return (out_line1.DotMul(in_line1) >= 0) && (out_line2.DotMul(in_line2) > 0);
}

Odom::Odom(Point3d pos, double speed, double yaw) :
  pos_(std::move(pos)), speed_(speed), yaw_(yaw) {}
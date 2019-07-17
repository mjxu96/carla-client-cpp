/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 16th July 2019 9:06:00 pm
 */

#ifndef MINJUN_COMMON_H_
#define MINJUN_COMMON_H_

#include "carla/client/Map.h"
#include "carla/geom/Location.h"
#include "carla/client/Waypoint.h"

#include <boost/shared_ptr.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <limits>

namespace minjun {

namespace utils {

class Point3d {
 public:
  Point3d() = default;
  Point3d(float x, float y, float z) : x_(x), y_(y), z_(z) {}
  float x_{0.0};
  float y_{0.0};
  float z_{0.0};

  bool operator==(const Point3d& p) const;


  std::string ToString() const;
};

class Line2d {
public:
  Line2d() = default;
  Line2d(const Point3d& p1, const Point3d& p2);
  Point3d start_;
  Point3d end_;

  double DotMul(const Line2d& l);
  double Distance(const Point3d& p, bool is_projection = true);
};

class MapUtils {
public:
  MapUtils(boost::shared_ptr<carla::client::Map> map_ptr);
  bool IsInJunction(const Point3d& p);
  size_t FindBehindIndex(const std::vector<Point3d>& points, const Point3d& current_point);


private:
  bool IsWithinSegment(const Point3d& p1, const Point3d& p2, const Point3d& p);
  bool IsWithinSegment(const Line2d& l, const Point3d& p);
  boost::shared_ptr<carla::client::Map> map_ptr_{nullptr};

};
  
} // namespace utils 
  
} // namespace minjun


namespace std {
template <>
struct hash<minjun::utils::Point3d> {
  size_t operator()(const minjun::utils::Point3d& p) const {
    return (hash<double>()(p.x_)) ^ (hash<double>()(p.y_)) ^
           (hash<double>()(p.z_));
  }
};
}  // namespace std

#endif
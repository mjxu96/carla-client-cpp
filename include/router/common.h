/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 8th July 2019 9:10:02 pm
 */

#ifndef MINJUN_ROUTER_COMMON_H_
#define MINJUN_ROUTER_COMMON_H_

#include "carla/client/Waypoint.h"

#include <boost/shared_ptr.hpp>
#include <boost/geometry.hpp>

#include <functional>
#include <string>
#include <limits>

namespace minjun {

class Point3d {
 public:
  Point3d() = default;
  Point3d(float x, float y, float z) : x_(x), y_(y), z_(z) {}
  float x_{0.0};
  float y_{0.0};
  float z_{0.0};
  bool operator==(const Point3d& p) const {
    return x_ == p.x_ && y_ == p.y_ && z_ == p.z_;
  }

  std::string ToString() const {
    return std::string("[") + std::to_string(x_) + std::string(", ") +
           std::to_string(y_) + std::string(", ") + std::to_string(z_) +
           std::string("]");
  }
};

class Node {
public:
  Node() = default;
  Node(boost::shared_ptr<carla::client::Waypoint> waypoint, double distance) :
    waypoint_(std::move(waypoint)), distance_(distance) {}
  boost::shared_ptr<carla::client::Waypoint> GetWaypoint() const {return waypoint_;}
  double GetDistance() const {return distance_;}
  
private:
  boost::shared_ptr<carla::client::Waypoint> waypoint_{nullptr};
  double distance_{std::numeric_limits<double>::max()};
};

struct NodeComparator {
  bool operator()(const Node& n1, const Node& n2) {
    if (n1.GetWaypoint()->GetId() == n2.GetWaypoint()->GetId()) {
      return false;
    } else {
      if (n1.GetDistance() == n2.GetDistance()) {
        return n1.GetWaypoint()->GetId() < n2.GetWaypoint()->GetId();
      }
    }
    return (n1.GetDistance() < n2.GetDistance());
  }
};

}  // namespace minjun

namespace std {
template <>
struct hash<minjun::Point3d> {
  size_t operator()(const minjun::Point3d& p) const {
    return (hash<double>()(p.x_)) ^ (hash<double>()(p.y_)) ^
           (hash<double>()(p.z_));
  }
};
}  // namespace std
#endif
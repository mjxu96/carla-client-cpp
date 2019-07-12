/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 8th July 2019 9:10:02 pm
 */

#ifndef MINJUN_ROUTER_COMMON_H_
#define MINJUN_ROUTER_COMMON_H_

#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/Waypoint.h"
#include "carla/client/World.h"

#include <boost/geometry.hpp>
#include <boost/shared_ptr.hpp>

#include <cmath>
#include <functional>
#include <limits>
#include <string>

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

  bool operator==(const Point3d& p) {
    return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                     std::pow((p.z_ - z_), 2.0)) < 0.01;
  }

  bool operator!=(const Point3d& p) { return !((*this) == p); }

  std::string ToString() const {
    return std::string("[") + std::to_string(x_) + std::string(", ") +
           std::to_string(y_) + std::string(", ") + std::to_string(z_) +
           std::string("]");
  }
};

class Node {
 public:
  Node() = default;
  Node(boost::shared_ptr<carla::client::Waypoint> waypoint, double distance)
      : waypoint_(std::move(waypoint)), distance_(distance) {}
  boost::shared_ptr<carla::client::Waypoint> GetWaypoint() const {
    return waypoint_;
  }
  double GetDistance() const { return distance_; }

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

namespace utils {

bool CompareRouterResults(const std::vector<Point3d>& p1s,
                          const std::vector<Point3d>& p2s) {
  if (p1s.size() != p2s.size()) {
    std::cout << "sizes are not the same" << std::endl;
    std::cout << "size 1: " << p1s.size() << "  size 2: " << p2s.size()
              << std::endl;
    return false;
  }
  for (size_t i = 0; i < p1s.size(); i++) {
    if (!(p1s[i] == p2s[i])) {
      std::cout << "No. " << i << " Point not same" << std::endl;
      return false;
    }
  }
  return true;
}

std::string DBGPrint(const carla::geom::Location& location) {
  return std::string("[") + std::to_string(location.x) + std::string(", ") +
         std::to_string(location.y) + std::string(", ") +
         std::to_string(location.z) + std::string("]");
}

std::vector<Point3d> ConvertFromWaypointToPoint3d(
    const std::vector<boost::shared_ptr<carla::client::Waypoint>>& waypoints) {
  std::vector<Point3d> result;
  for (const auto& waypoint_ptr : waypoints) {
    auto location = waypoint_ptr->GetTransform().location;
    result.emplace_back(location.x, location.y, location.z);
  }
  return result;
}

double Distance(const carla::geom::Location& p1,
                const carla::geom::Location& p2) {
  return std::sqrt(std::pow((p1.x - p2.x), 2.0) + std::pow((p1.y - p2.y), 2.0) +
                   std::pow((p1.z - p2.z), 2.0));
}

double Distance(const carla::client::Waypoint& p1,
                const carla::client::Waypoint& p2) {
  return Distance(p1.GetTransform().location, p2.GetTransform().location);
}

double Distance(const boost::shared_ptr<carla::client::Waypoint>& p1,
                const boost::shared_ptr<carla::client::Waypoint>& p2) {
  return Distance(*p1, *p2);
}

double Distance(const Point3d& p1, const Point3d& p2) {
  return std::sqrt(std::pow((p1.x_ - p2.x_), 2.0) +
                   std::pow((p1.y_ - p2.y_), 2.0) +
                   std::pow((p1.z_ - p2.z_), 2.0));
}

void SetAllTrafficLightToBeGreen(const carla::client::World& world_ptr) {
  auto actors = world_ptr.GetActors();
  for (const auto& actor : *actors) {
    if (actor->GetTypeId() == "traffic.traffic_light") {
      (boost::static_pointer_cast<carla::client::TrafficLight>(actor))
          ->SetState(carla::rpc::TrafficLightState::Green);
      (boost::static_pointer_cast<carla::client::TrafficLight>(actor))
          ->SetGreenTime(100.0);
    }
  }
}

}  // namespace utils

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
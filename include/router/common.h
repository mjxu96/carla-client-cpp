/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 8th July 2019 9:10:02 pm
 */

#ifndef MINJUN_ROUTER_COMMON_H_
#define MINJUN_ROUTER_COMMON_H_

#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/Map.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/Waypoint.h"
#include "carla/client/World.h"

#include <boost/geometry.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <limits>
#include <random>
#include <string>

namespace minjun {

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

class Node {
 public:
  Node() = default;
  Node(boost::shared_ptr<carla::client::Waypoint> waypoint, double distance);
  boost::shared_ptr<carla::client::Waypoint> GetWaypoint() const;
  double GetDistance() const;

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
                          const std::vector<Point3d>& p2s);

std::string DBGPrint(const carla::geom::Location& location);

std::vector<Point3d> ConvertFromWaypointToPoint3d(
    const std::vector<boost::shared_ptr<carla::client::Waypoint>>& waypoints);

double Distance(const carla::geom::Location& p1,
                const carla::geom::Location& p2);

double Distance(const carla::client::Waypoint& p1,
                const carla::client::Waypoint& p2);

double Distance(const boost::shared_ptr<carla::client::Waypoint>& p1,
                const boost::shared_ptr<carla::client::Waypoint>& p2);

double Distance(const Point3d& p1, const Point3d& p2);

void SetAllTrafficLightToBeGreen(const carla::client::World& world_ptr);

class RRTUtils {
 public:
  RRTUtils(boost::shared_ptr<carla::client::Map> map_ptr,
           const Point3d& start_point, const Point3d& end_point,
           double road_offset_threshold = 3.0)
      : map_ptr_(std::move(map_ptr)),
        start_point_(std::move(start_point)),
        end_point_(std::move(end_point)),
        road_offset_threshold_(road_offset_threshold) {
    auto random_waypoints = map_ptr_->GenerateWaypoints(road_offset_threshold_);
    for (const auto& waypoint : random_waypoints) {
      auto loc = waypoint->GetTransform().location;
      random_points_.emplace_back(loc.x, loc.y, loc.z);
    }
    std::srand(std::time(nullptr));
  }

  bool IsInRoad(const Point3d& p) {
    carla::geom::Location p_loc(p.x_, p.y_, p.z_);
    auto waypoint = map_ptr_->GetWaypoint(p_loc);
    if (Distance(waypoint->GetTransform().location, p_loc) >=
        road_offset_threshold_) {
      // std::cout << "distance: " <<
      // Distance(waypoint->GetTransform().location, p_loc) << std::endl;
      return false;
    }
    return true;
  }

  std::shared_ptr<Point3d> RandomSample(double toward_pro = 0.3) {
    double random_value = std::rand() / ((double)RAND_MAX);
    // std::cout << random_value << std::endl;

    if (random_value < toward_pro) {
      return std::make_shared<Point3d>(end_point_);
    } else {
      return std::make_shared<Point3d>(
          random_points_[std::rand() % random_points_.size()]);
    }
  }

  std::shared_ptr<Point3d> Extend(const Point3d& cur, const Point3d& tar,
                                  double delta_q = 3.0) {
    double angle = std::atan2(tar.y_ - cur.y_, tar.x_ - cur.x_);
    double delta_x = delta_q * std::cos(angle);
    double delta_y = delta_q * std::sin(angle);
    std::shared_ptr<Point3d> res =
        std::make_shared<Point3d>(delta_x + cur.x_, delta_y + cur.y_, cur.z_);
    if (!IsInRoad(*res)) {
      return nullptr;
    }
    return res;
  }

  static size_t FindNearestIndex(
      const std::vector<std::shared_ptr<Point3d>>& points, const Point3d& p) {
    if (points.size() == 0) {
      std::cerr << "ERROR! points size cannot be 0!!!" << std::endl;
      return 0;
    }

    double min_distance = std::numeric_limits<double>::max();
    size_t index = 0;

    for (size_t i = 0; i < points.size(); i++) {
      if (Distance(p, *(points[i])) < min_distance) {
        index = i;
        min_distance = Distance(p, *(points[i]));
      }
    }

    return index;
  }

 private:
  boost::shared_ptr<carla::client::Map> map_ptr_{nullptr};
  Point3d start_point_;
  Point3d end_point_;
  std::vector<Point3d> random_points_{};
  double road_offset_threshold_{3.0};
};

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
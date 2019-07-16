/*
 * File: router.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 8th July 2019 9:06:01 pm
 */

#ifndef MINJUN_ROUTER_H_
#define MINJUN_ROUTER_H_

#include "router/common.h"

#include "carla/client/Actor.h"
#include "carla/client/ActorList.h"
#include "carla/client/BlueprintLibrary.h"
#include "carla/client/Client.h"
#include "carla/client/Map.h"
#include "carla/client/TrafficLight.h"
#include "carla/client/Waypoint.h"
#include "carla/client/World.h"
#include "carla/geom/Location.h"

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <queue>
#include <set>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

namespace minjun {

class Router {
 public:
  Router(Point3d start_point, Point3d end_point,
         boost::shared_ptr<carla::client::Map> map_ptr);

  std::vector<Point3d> GetRoutePoints();
  void SetPointInterval(double interval);

  // debug use
  void DbgSetActor(boost::shared_ptr<carla::client::Actor> actor);

 private:
  std::vector<Point3d> RRT();
  std::vector<Point3d> Dijkstra();
  std::vector<Point3d> AStar();
  std::vector<Point3d> BFS();
  boost::shared_ptr<carla::client::Map> map_ptr_{nullptr};
  Point3d start_point_;
  Point3d end_point_;

  double point_interval_{2};
  double distance_threshold_{20.0};

  // debug use
  boost::shared_ptr<carla::client::Actor> actor_{nullptr};
};

}  // namespace minjun

#endif
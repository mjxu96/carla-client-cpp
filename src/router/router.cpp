/*
 * File: router.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 8th July 2019 9:05:53 pm
 */

#include "router/router.h"

using namespace minjun;
using namespace std::chrono_literals;
using namespace std::string_literals;

double Distance(const Point3d& p1, const Point3d& p2) {
  return std::sqrt(std::pow((p1.x_ - p2.x_), 2.0) +
                   std::pow((p1.y_ - p2.y_), 2.0) +
                   std::pow((p1.z_ - p2.z_), 2.0) );
}

Router::Router(Point3d start_point, Point3d end_point,
               boost::shared_ptr<carla::client::Map> map_ptr)
    : start_point_(std::move(start_point)), end_point_(std::move(end_point)),
      map_ptr_(std::move(map_ptr)) {}

void Router::SetPointInterval(double interval) {
  point_interval_ = interval;
}

std::vector<Point3d> Router::GetRoutePoints() {
  return BFS();
}

std::vector<Point3d> Router::BFS() {
  std::unordered_set<uint32_t> traveled_points;
  std::queue<std::pair<Point3d, uint32_t>> point_queue;
  point_queue.push({start_point_, 0});
  while (!point_queue.empty()) {
    auto current_point = point_queue.front().first;
    uint32_t current_distance = point_queue.front().second;
    point_queue.pop();
    if (Distance(end_point_, current_point) <= distance_threshold_) {
      std::cout << "get destination" << std::endl;
      std::cout << current_distance << std::endl;
      break;
    }
    carla::geom::Location location(current_point.x_, current_point.y_, current_point.z_);
    auto current_waypoint = map_ptr_->GetWaypoint(location);
    auto waypoints = current_waypoint->GetNext(point_interval_);
    for (const auto& waypoint : waypoints) {
      if (traveled_points.find(waypoint->GetId()) != traveled_points.end()) {
        continue;
      }
      traveled_points.insert(waypoint->GetId());
      Point3d point((*waypoint).GetTransform().location.x,
                    (*waypoint).GetTransform().location.y,  
                    (*waypoint).GetTransform().location.z);  
      point_queue.push({point, current_distance + 1});
    }
    return std::vector<Point3d>();
  }
}

int main(int argc, char* argv[]) {
  std::string host(argv[1]);
  uint16_t port(2000u);
  auto client = carla::client::Client(host, port);
  client.SetTimeout(10s);
  auto world = client.GetWorld();
  auto map = world.GetMap();
  auto transforms = (*map).GetRecommendedSpawnPoints();
  std::cout << "recommended points size: " << transforms.size() << std::endl;
  auto size = transforms.size();
  Point3d p1(transforms[0].location.x,
             transforms[0].location.y,
             transforms[0].location.z);
  Point3d p2(transforms[size/2].location.x,
             transforms[size/2].location.y,
             transforms[size/2].location.z);
  std::cout << "start point: " << p1.ToString() << std::endl;
  std::cout << "end point: " << p2.ToString() << std::endl;
  Router router(p1, p2, map);
  router.GetRoutePoints();
  return 0;
}
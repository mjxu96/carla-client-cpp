/*
 * File: router.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 8th July 2019 9:05:53 pm
 */

#include "router/router.h"

using namespace minjun;
using namespace std::chrono_literals;
using namespace std::string_literals;

std::string DBGPrint(const carla::geom::Location& location) {
    return std::string("[") + std::to_string(location.x) + std::string(", ") +
                              std::to_string(location.y) + std::string(", ") +
                              std::to_string(location.z) + std::string("]");
  
}

std::vector<Point3d> ConvertFromWaypointToPoint3d(const std::vector<boost::shared_ptr<carla::client::Waypoint>>& waypoints) {
  std::vector<Point3d> result;
  for (const auto& waypoint_ptr : waypoints) {
    auto location = waypoint_ptr->GetTransform().location;
    result.emplace_back(location.x, location.y, location.z);
  }
  return result;
}

double Distance(const carla::geom::Location& p1, const carla::geom::Location& p2) {
  return std::sqrt(std::pow((p1.x - p2.x), 2.0) +
                   std::pow((p1.y - p2.y), 2.0) +
                   std::pow((p1.z - p2.z), 2.0) );

}

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
  std::vector<Point3d> result;
  std::unordered_set<uint64_t> travelled_points;
  std::queue<std::pair<boost::shared_ptr<carla::client::Waypoint>, std::vector<boost::shared_ptr<carla::client::Waypoint>>>> point_queue;
  auto start_waypoint = map_ptr_->GetWaypoint(carla::geom::Location(start_point_.x_, start_point_.y_, start_point_.z_));
  auto end_waypoint = map_ptr_->GetWaypoint(carla::geom::Location(end_point_.x_, end_point_.y_, end_point_.z_));
  auto end_lane_id = end_waypoint->GetLaneId();
  auto end_location = carla::geom::Location(end_point_.x_, end_point_.y_, end_point_.z_);
  point_queue.push({start_waypoint, std::vector<boost::shared_ptr<carla::client::Waypoint>>()});
  while (!point_queue.empty()) {
    auto current_waypoint = point_queue.front().first;
    auto current_waypoints = point_queue.front().second;
    point_queue.pop();
    if (Distance(current_waypoint->GetTransform().location, end_location) < distance_threshold_) {
      auto current_lane_id = current_waypoint->GetLaneId();
      if (current_lane_id == end_lane_id) {
        std::cout << "size: " << current_waypoints.size() << std::endl;
        return ConvertFromWaypointToPoint3d(current_waypoints);
      }
    }
    auto next_waypoints = current_waypoint->GetNext(point_interval_);
    for (const auto& next_waypoint : next_waypoints) {
      uint64_t id = next_waypoint->GetId();
      if (travelled_points.find(id) != travelled_points.end()) {
        continue;
      }
      travelled_points.insert(id);
      auto current_waypoints_copy = current_waypoints;
      current_waypoints_copy.push_back(next_waypoint);
      point_queue.push({next_waypoint, current_waypoints_copy});
    }
  }
  return result;
}
      /*
std::vector<Point3d> Router::BFS() {
  int count = 0;
  std::vector<Point3d> tmp;
  std::unordered_map<uint64_t, Point3d> traveled_points;
  std::queue<std::pair<Point3d, std::vector<Point3d>>> point_queue;
  point_queue.push({start_point_, std::vector<Point3d>()});
  std::cout << "initial distance: " << Distance(start_point_, end_point_) << std::endl;
  while (!point_queue.empty()) {
    auto current_point = point_queue.front().first;
    auto current_points = point_queue.front().second;
    point_queue.pop();
    if (Distance(end_point_, current_point) <= distance_threshold_) {
      std::cout << "get destination" << std::endl;
      std::cout << current_points.size() << std::endl;
      return tmp;
      return current_points;
    }
    carla::geom::Location location(current_point.x_, current_point.y_, current_point.z_);
    auto current_waypoint = map_ptr_->GetWaypoint(location);
    auto waypoints = current_waypoint->GetNext(point_interval_);
    if (waypoints.size() > 1) {
      std::cout << "location : " << DBGPrint(location) << std::endl;
      std::cout << "currentw : " << DBGPrint(current_waypoint->GetTransform().location) << std::endl;
      std::cout << "waypoint1: " << DBGPrint(waypoints[0]->GetTransform().location) << std::endl;
      std::cout << "waypoint2: " << DBGPrint(waypoints[1]->GetTransform().location) << std::endl;
      double s1 = 0.0, s2 = 0.0;
      auto p1_p = waypoints[0]->GetTransform().location;
      auto p2_p = waypoints[1]->GetTransform().location;
      auto waypoint1 = waypoints[0];
      auto waypoint2 = waypoints[1];
      for (int i = 0; i < 20; i++) {
        waypoint1 = map_ptr_->GetWaypoint(p1_p)->GetNext(point_interval_)[0];// waypoint1->GetNext(point_interval_)[0];
        s1 += Distance(p1_p, waypoint1->GetTransform().location);
        waypoint2 = map_ptr_->GetWaypoint(p2_p)->GetNext(point_interval_)[0];// waypoint2->GetNext(point_interval_)[0];
        s2 += Distance(p2_p, waypoint2->GetTransform().location);
        p1_p = waypoint1->GetTransform().location;
        p2_p = waypoint2->GetTransform().location;
      }
      std::cout << "s1: " << s1 << ", s2: " << s2 << std::endl;
      std::cout << "waypoint1 next waypoint: " << DBGPrint(waypoints[0]->GetNext(point_interval_)[0]->GetTransform().location) << std::endl;
      std::cout << "waypoint2 next waypoint: " << DBGPrint(waypoints[1]->GetNext(point_interval_)[0]->GetTransform().location) << std::endl;
      std::cout << "waypoint1 next next waypoint: " << DBGPrint(waypoints[0]->GetNext(point_interval_)[0]->GetNext(point_interval_)[0]->GetTransform().location) << std::endl;
      std::cout << "waypoint2 next next waypoint: " << DBGPrint(waypoints[1]->GetNext(point_interval_)[0]->GetNext(point_interval_)[0]->GetTransform().location) << std::endl;
      std::cout << "waypoint1 next next next waypoint: " << DBGPrint(waypoints[0]->GetNext(point_interval_)[0]->GetNext(point_interval_)[0]->GetNext(point_interval_)[0]->GetTransform().location) << std::endl;
      std::cout << "waypoint2 next next next waypoint: " << DBGPrint(waypoints[1]->GetNext(point_interval_)[0]->GetNext(point_interval_)[0]->GetNext(point_interval_)[0]->GetTransform().location) << std::endl;
      std::cout << "currentw road id: " << ((*current_waypoint).GetRoadId()) << std::endl;
      std::cout << "waypoint1 road id: " << ((*waypoints[0]).GetRoadId()) << std::endl;
      std::cout << "waypoint2 road id: " << ((*waypoints[1]).GetRoadId()) << std::endl;
      std::cout << "waypoint1 id == waypoint2 id? " << (waypoints[0]->GetId() == waypoints[1]->GetId() ? "true" : "false") << std::endl;
    }
    for (const auto& waypoint : waypoints) {
      if (traveled_points.find(waypoint->GetId()) != traveled_points.end()) {
        continue;
      }
      Point3d point((*waypoint).GetTransform().location.x,
                    (*waypoint).GetTransform().location.y,  
                    (*waypoint).GetTransform().location.z);  
      auto current_points_copy = current_points;
      traveled_points.insert({waypoint->GetId(), point});
      current_points_copy.push_back(point);
      point_queue.push({point, current_points_copy});
    }
  }
  return std::vector<Point3d>();
}
      */

void SetAllTrafficLightToBeGreen(const carla::client::World& world_ptr) {
  auto actors = world_ptr.GetActors();
  for (const auto& actor : *actors) {
    //std::cout << actor->GetTypeId() << std::endl;
    if (actor->GetTypeId() == "traffic.traffic_light") {
      (boost::static_pointer_cast<carla::client::TrafficLight>(actor))->SetState(carla::rpc::TrafficLightState::Green);
      (boost::static_pointer_cast<carla::client::TrafficLight>(actor))->SetGreenTime(100.0);
    }
  }
}

int main(int argc, char* argv[]) {
  std::string host(argv[1]);
  uint16_t port(2000u);
  auto client = carla::client::Client(host, port);
  client.SetTimeout(10s);
  auto world = client.GetWorld();
  auto map = world.GetMap();
  auto blueprint_library = world.GetBlueprintLibrary();
  auto vehicles = blueprint_library->Filter("vehicle");
  auto vehicle_bp = (*vehicles)[0];

  auto transforms = (*map).GetRecommendedSpawnPoints();
  std::cout << "recommended points size: " << transforms.size() << std::endl;
  auto size = transforms.size();

  SetAllTrafficLightToBeGreen(world);

  size_t p1_i = 0;
  size_t p2_i = size - 1;
  Point3d p1(transforms[p1_i].location.x,
             transforms[p1_i].location.y,
             transforms[p1_i].location.z);
  Point3d p2(transforms[p2_i].location.x,
             transforms[p2_i].location.y,
             transforms[p2_i].location.z);
  std::cout << "start point: " << p1.ToString() << std::endl;
  std::cout << "end point: " << p2.ToString() << std::endl;
  auto vehicle1 = world.SpawnActor(vehicle_bp, transforms[p1_i]);
  auto vehicle2 = world.SpawnActor(vehicle_bp, transforms[p2_i]);
  vehicle1->SetSimulatePhysics(false);
  Router router(p1, p2, map);
  auto points = router.GetRoutePoints();
  //std::cout << "size: " << points.size() << std::endl;
  for (const auto& point : points) {
    vehicle1->SetLocation(carla::geom::Location(point.x_, point.y_, point.z_));
    std::this_thread::sleep_for(50ms);
    //std::cout << point.ToString() << std::endl;
  }
  std::cin.ignore();
  std::cin.get();
  vehicle1->Destroy();
  vehicle2->Destroy();
  return 0;
}
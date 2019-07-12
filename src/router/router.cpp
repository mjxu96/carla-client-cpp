/*
 * File: router.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 8th July 2019 9:05:53 pm
 */

#include "router/router.h"

using namespace minjun;
using namespace minjun::utils;
using namespace std::chrono_literals;
using namespace std::string_literals;

Router::Router(Point3d start_point, Point3d end_point,
               boost::shared_ptr<carla::client::Map> map_ptr)
    : start_point_(std::move(start_point)),
      end_point_(std::move(end_point)),
      map_ptr_(std::move(map_ptr)) {}

void Router::SetPointInterval(double interval) { point_interval_ = interval; }

std::vector<Point3d> Router::GetRoutePoints() {
  auto astar_start = std::chrono::system_clock::now();
  auto res1 = AStar();
  auto astar_end = std::chrono::system_clock::now();
  auto res2 = BFS();
  if (CompareRouterResults(res1, res2)) {
    std::cout << "results same" << std::endl;
  }
  auto bfs_end = std::chrono::system_clock::now();
  std::chrono::duration<double> astar_duration = astar_end - astar_start;
  std::chrono::duration<double> bfs_duration = bfs_end - astar_end;
  auto astart_time = astar_duration.count();
  auto bfs_time = bfs_duration.count();
  std::cout << "Astar time: " << astart_time << std::endl;
  std::cout << "BFS time: " << bfs_time << std::endl;
  return res1;
  // return BFS();
}

std::vector<Point3d> Router::AStar() {
  std::set<Node, NodeComparator> open_set;
  boost::unordered_map<boost::shared_ptr<carla::client::Waypoint>,
                       boost::shared_ptr<carla::client::Waypoint>>
      waypoint_predecessor;
  boost::unordered_map<boost::shared_ptr<carla::client::Waypoint>, double>
      g_score;
  auto start_waypoint = map_ptr_->GetWaypoint(
      carla::geom::Location(start_point_.x_, start_point_.y_, start_point_.z_));
  auto end_waypoint = map_ptr_->GetWaypoint(
      carla::geom::Location(end_point_.x_, end_point_.y_, end_point_.z_));
  g_score.insert({start_waypoint, 0.0});
  Node start_node(start_waypoint, Distance(start_waypoint, end_waypoint));
  open_set.insert(start_node);

  while (!open_set.empty()) {
    auto current_node_it = open_set.begin();
    auto current_waypoint = current_node_it->GetWaypoint();
    open_set.erase(current_node_it);
    if (Distance(end_waypoint, current_waypoint) < distance_threshold_) {
      std::cout << "found" << std::endl;
      std::vector<boost::shared_ptr<carla::client::Waypoint>> result_waypoints;
      while (waypoint_predecessor.find(current_waypoint) !=
             waypoint_predecessor.end()) {
        result_waypoints.push_back(current_waypoint);
        current_waypoint =
            (waypoint_predecessor.find(current_waypoint)->second);
      }
      auto result = ConvertFromWaypointToPoint3d(result_waypoints);
      std::reverse(result.begin(), result.end());
      return result;
    }

    auto next_waypoints = current_waypoint->GetNext(point_interval_);
    for (const auto& next_waypoint : next_waypoints) {
      double tmp_g_score =
          g_score[current_waypoint] + Distance(next_waypoint, current_waypoint);
      if (g_score.find(next_waypoint) == g_score.end() ||
          g_score[next_waypoint] > tmp_g_score) {
        g_score[next_waypoint] = tmp_g_score;
        waypoint_predecessor[next_waypoint] = current_waypoint;
        Node next_node(next_waypoint,
                       tmp_g_score + Distance(end_waypoint, next_waypoint));
        auto next_node_it = open_set.find(next_node);
        if (next_node_it != open_set.end()) {
          open_set.erase(next_node_it);
        }
        open_set.insert(next_node);
      }
    }
  }

  std::vector<Point3d> result;
  return result;
}

std::vector<Point3d> Router::BFS() {
  std::vector<Point3d> result;
  std::unordered_set<uint64_t> travelled_points;
  std::queue<std::pair<boost::shared_ptr<carla::client::Waypoint>,
                       std::vector<boost::shared_ptr<carla::client::Waypoint>>>>
      point_queue;
  auto start_waypoint = map_ptr_->GetWaypoint(
      carla::geom::Location(start_point_.x_, start_point_.y_, start_point_.z_));
  auto end_waypoint = map_ptr_->GetWaypoint(
      carla::geom::Location(end_point_.x_, end_point_.y_, end_point_.z_));
  auto end_lane_id = end_waypoint->GetLaneId();
  auto end_road_id = end_waypoint->GetRoadId();
  auto end_location =
      carla::geom::Location(end_point_.x_, end_point_.y_, end_point_.z_);
  point_queue.push({start_waypoint,
                    std::vector<boost::shared_ptr<carla::client::Waypoint>>()});
  while (!point_queue.empty()) {
    auto current_waypoint = point_queue.front().first;
    auto current_waypoints = point_queue.front().second;
    point_queue.pop();
    if (Distance(current_waypoint->GetTransform().location, end_location) <
        distance_threshold_) {
      // auto current_lane_id = current_waypoint->GetLaneId();
      // auto current_road_id = current_waypoint->GetRoadId();
      // if (current_lane_id == end_lane_id && current_road_id == end_road_id) {
      return ConvertFromWaypointToPoint3d(current_waypoints);
      //}
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

void Router::DbgSetActor(boost::shared_ptr<carla::client::Actor> actor) {
  actor_ = std::move(actor);
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "error, not enough params" << std::endl;
  }
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
  auto size = transforms.size();

  SetAllTrafficLightToBeGreen(world);

  size_t p1_i = 0;
  size_t p2_i = size / 2;
  if (argc >= 3) {
    p1_i = std::stoi(argv[2]) % transforms.size();
    if (argc >= 4) {
      p2_i = std::stoi(argv[3]) % transforms.size();
    }
  }
  Point3d p1(transforms[p1_i].location.x, transforms[p1_i].location.y,
             transforms[p1_i].location.z);
  Point3d p2(transforms[p2_i].location.x, transforms[p2_i].location.y,
             transforms[p2_i].location.z);
  std::cout << "start point: " << p1.ToString() << std::endl;
  std::cout << "end point: " << p2.ToString() << std::endl;
  auto vehicle1 = world.SpawnActor(vehicle_bp, transforms[p1_i]);
  auto vehicle2 = world.SpawnActor(vehicle_bp, transforms[p2_i]);
  vehicle1->SetSimulatePhysics(false);
  Router router(p1, p2, map);

  // dbg use
  // router.DbgSetActor(vehicle1);

  auto points = router.GetRoutePoints();

  for (const auto& point : points) {
    vehicle1->SetLocation(carla::geom::Location(point.x_, point.y_, point.z_));
    std::this_thread::sleep_for(50ms);
  }
  std::cin.ignore();
  std::cin.get();
  vehicle1->Destroy();
  vehicle2->Destroy();
  return 0;
}
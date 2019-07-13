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
  auto astar_res = AStar();
  auto astar_end = std::chrono::system_clock::now();
  auto bfs_res = BFS();
  auto bfs_end = std::chrono::system_clock::now();
  auto dijkstra_res = Dijkstra();
  auto dijkstra_end = std::chrono::system_clock::now(); 
  auto rrt_res = RRT();
  auto rrt_end = std::chrono::system_clock::now(); 
  if (CompareRouterResults(astar_res, bfs_res)) {
    std::cout << "astar and bfs results same" << std::endl;
  }
  if (CompareRouterResults(dijkstra_res, bfs_res)) {
    std::cout << "dijkstra and bfs results same" << std::endl;
  }
  std::chrono::duration<double> astar_duration = astar_end - astar_start;
  std::chrono::duration<double> bfs_duration = bfs_end - astar_end;
  std::chrono::duration<double> dijkstra_duration = dijkstra_end - bfs_end;
  std::chrono::duration<double> rrt_duration = rrt_end -  dijkstra_end;
  auto astart_time = astar_duration.count();
  auto bfs_time = bfs_duration.count();
  auto dijkstra_time = dijkstra_duration.count();
  auto rrt_time = rrt_duration.count();
  std::cout << "Astar time:    " << astart_time << std::endl;
  std::cout << "BFS time:      " << bfs_time << std::endl;
  std::cout << "Dijkstra time: " << dijkstra_time << std::endl;
  std::cout << "RRT time:      " << rrt_time << std::endl;
  return dijkstra_res;
}

std::vector<Point3d> Router::RRT() {
  RRTUtils rrt_utils(map_ptr_, start_point_, end_point_);
  std::unordered_map<std::shared_ptr<Point3d>, std::shared_ptr<Point3d>> predecessor_points;
  std::vector<std::shared_ptr<Point3d>> discorvered_points;
  std::vector<Point3d> result;
  discorvered_points.push_back(std::make_shared<Point3d>(start_point_));

  size_t count = 0;
  size_t times_limit = 100000;
  while (count < times_limit) {
    auto point = rrt_utils.RandomSample(); // default probability is 0.3
    size_t nearest_index = rrt_utils.FindNearestIndex(discorvered_points, *point);
    auto new_point = rrt_utils.Extend(*(discorvered_points[nearest_index]), *point);
    if (new_point != nullptr) {
      discorvered_points.push_back(new_point);
      predecessor_points[new_point] = discorvered_points[nearest_index];
      if (Distance(end_point_, *new_point) < distance_threshold_) {
        std::cout << "RRT found" << std::endl;
        while (predecessor_points.find(new_point) != predecessor_points.end()) {
          result.push_back(*new_point);
          new_point = predecessor_points.find(new_point)->second;
        }
        std::reverse(result.begin(), result.end());
        return result;
      }
    }

    count++;
  }

  std::cout << "RRT not found" << std::endl;
  return result;
}

std::vector<Point3d> Router::Dijkstra() {
  std::set<Node, NodeComparator> open_set;
  boost::unordered_map<boost::shared_ptr<carla::client::Waypoint>,
                       boost::shared_ptr<carla::client::Waypoint>>
      waypoint_predecessor;
  auto start_waypoint = map_ptr_->GetWaypoint(
      carla::geom::Location(start_point_.x_, start_point_.y_, start_point_.z_));
  auto end_waypoint = map_ptr_->GetWaypoint(
      carla::geom::Location(end_point_.x_, end_point_.y_, end_point_.z_));
  Node start_node(start_waypoint, 0.0);
  open_set.insert(start_node);

  while (!open_set.empty()) {
    auto current_node_it = open_set.begin();
    auto current_waypoint = current_node_it->GetWaypoint();
    double current_distance = current_node_it->GetDistance();
    open_set.erase(current_node_it);

    if (Distance(end_waypoint, current_waypoint) < distance_threshold_) {
      std::cout << "Dijkstra found" << std::endl;
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
      double tmp_distance = current_distance + Distance(current_waypoint, next_waypoint);
      Node next_node(next_waypoint, tmp_distance);
      auto next_node_it = open_set.find(next_node);
      if (next_node_it == open_set.end()) {
        open_set.insert(next_node);
        waypoint_predecessor[next_waypoint] =  current_waypoint;
      } else {
        if (next_node_it->GetDistance() > tmp_distance) {
          open_set.erase(next_node_it);
          waypoint_predecessor[next_waypoint] =  current_waypoint;
          open_set.insert(next_node);
        }
      }
    }
  }
  std::cout << "Dijkstra not found" << std::endl;
  std::vector<Point3d> result;
  return result;
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
      std::cout << "A* found" << std::endl;
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

  std::cout << "A* not found" << std::endl;
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
      std::cout << "BFS found" << std::endl;
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
  std::cout << "BFS not found" << std::endl;
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
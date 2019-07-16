
#include "router/common.h"
namespace minjun {
  


bool Point3d::operator==(const Point3d& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0)) < 0.001;
}
std::string Point3d::ToString() const {
  return std::string("[") + std::to_string(x_) + std::string(", ") +
         std::to_string(y_) + std::string(", ") + std::to_string(z_) +
         std::string("]");
}

Node::Node(boost::shared_ptr<carla::client::Waypoint> waypoint, double distance)
      : waypoint_(std::move(waypoint)), distance_(distance) {}
boost::shared_ptr<carla::client::Waypoint> Node::GetWaypoint() const {
  return waypoint_;
}

double Node::GetDistance() const { return distance_; }

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

std::vector<Point3d> ConvertFromWaypointToPoint3d(
    const std::vector<boost::shared_ptr<carla::client::Waypoint>>& waypoints);


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


} // namespace utils
} // namespace minjun
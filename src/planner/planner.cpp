#include "planner/planner.h"

using namespace minjun;
using namespace minjun::utils;
using namespace minjun::planner;

using namespace std::chrono_literals;
using namespace std::string_literals;

Planner::Planner(Point3d start_point, Point3d end_point,
                 boost::shared_ptr<carla::client::Map> map_ptr, double init_yaw,
                 double init_speed)
    : start_point_(std::move(start_point)),
      end_point_(std::move(end_point)),
      map_ptr_(std::move(map_ptr)),
      init_yaw_(init_yaw),
      init_speed_(init_speed),
      map_utils_(map_ptr_) {
  Router router(start_point_, end_point_, map_ptr_);
  router_points_ = router.GetRoutePoints(1.0);

  // find points pair that are in the junction
  for (size_t i = 0; i < router_points_.size(); i++) {
    auto point = router_points_[i];
    if (!map_utils_.IsInJunction(point)) {
      continue;
    }
    size_t first = i;
    size_t second = i + 1;
    while (second < router_points_.size() && map_utils_.IsInJunction(router_points_[second])) {
      second++;
    }
    junction_points_pair_.push_back({first, second-1});
    i = second;
  }

}

std::vector<PlannerPoint> Planner::GetPlannerPoints() {
  std::vector<PlannerPoint> planner_points;
  double pre_yaw = init_yaw_;
  double pre_speed = init_speed_;
  const double acce_limit = constants::Constants::acceleration;
  const double speed_limit = constants::Constants::speed;
  const double yaw_rate_limit = constants::Constants::yaw_rate;
  size_t point_num = router_points_.size();
  if (point_num == 0) {
    return planner_points;
  }

  for (size_t i = 0; i < point_num - 1; i++) {
    planner_points.emplace_back(router_points_[i], pre_speed, pre_yaw);

    double s = Distance(router_points_[i], router_points_[i + 1]);
    double target_yaw = Utils::GetYaw(router_points_[i], router_points_[i + 1]);
    auto t_option = Utils::SolveFunction(0.5 * acce_limit, pre_speed, -s);
    if (t_option == boost::none) {
      LOG_ERROR("Something error when solve time function");
      continue;
    }
    auto t = std::max(t_option.value().first, t_option.value().second);
    if (t < 0) {
      LOG_ERROR("Something error when solve time function");
      continue;
    }

    double tmp_pre_speed = pre_speed;
    pre_speed += t * acce_limit;
    if (pre_speed > speed_limit) {
      pre_speed = speed_limit;
      double tmp_t = (speed_limit - tmp_pre_speed) / acce_limit;
      double s_left = s - tmp_t * (tmp_pre_speed + speed_limit) / 2.0;
      t = tmp_t + s_left / speed_limit;
    }
    pre_yaw = Utils::AfterRotate(pre_yaw, target_yaw, t, yaw_rate_limit);
  }

  planner_points.emplace_back(router_points_[point_num - 1], pre_speed,
                              pre_yaw);
  return planner_points;
}

std::vector<PlannerPoint> Planner::GetPlannerPoints(const Point3d& current_pos) {
  std::vector<PlannerPoint> planner_points;
  map_utils_.FindBehindIndex(router_points_, router_points_[0]);
  for (size_t i = 0; i < router_points_.size(); i++) {
    auto tmp_pos = router_points_[i];
    if (i != map_utils_.FindBehindIndex(router_points_, tmp_pos)) {
      std::cout << "something wrong with find index, true i: " << i << " result: " <<  map_utils_.FindBehindIndex(router_points_, tmp_pos) << std::endl;
    }
  }
  return planner_points;
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
  auto init_yaw = vehicle1->GetTransform().rotation.yaw;
  Planner planner(p1, p2, map, init_yaw, 0.0);

  // dbg use
  // router.DbgSetActor(vehicle1);

  auto points = planner.GetPlannerPoints();
  planner.GetPlannerPoints(p1);


  // for (const auto& point : points) {
  //   auto router_point = point.GetPoint();
  //   carla::geom::Rotation rotation(0.0, point.GetYaw(), 0.0);
  //   carla::geom::Transform transfrom(
  //       carla::geom::Location(router_point.x_, router_point.y_,
  //                             router_point.z_),
  //       rotation);
  //   vehicle1->SetTransform(transfrom);
  //   std::this_thread::sleep_for(50ms);
  // }
  std::cin.ignore();
  std::cin.get();
  vehicle1->Destroy();
  vehicle2->Destroy();
  return 0;
}
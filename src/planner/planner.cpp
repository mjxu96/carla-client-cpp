#include "planner/planner.h"

using namespace minjun;
using namespace minjun::utils;
using namespace minjun::planner;


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
  router_points_ = router.GetRoutePoints(router_point_interval_);

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

  // look ahead set
  look_ahead_index_ = static_cast<size_t>(look_ahead_distance_ / router_point_interval_);

}
std::vector<Point3d> Planner::GetRouterPoints() const {
  return router_points_;
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

std::vector<PlannerPoint> Planner::GetPlannerPoints(const Odom& current_odom) {
  planner_points_.emplace_back(current_odom.GetPosition(), current_odom.GetSpeed(), current_odom.GetYaw());
  std::vector<PlannerPoint> planner_points;
  size_t current_index = map_utils_.FindBehindIndex(router_points_, current_odom.GetPosition());
  std::cout << "current idx: " << current_index << std::endl;
  if (current_index >= router_points_.size() - 1) {
    return planner_points;
  }

  auto curr_point = current_odom.GetPosition();
  auto curr_speed = current_odom.GetSpeed();
  auto curr_yaw = current_odom.GetYaw();
  for (size_t i = current_index + 1; i < router_points_.size() - 1; i++) {
    // Actually I dont know why I put this if condition here when I wrote this planner....
    // if ((curr_speed == 0.0 || curr_yaw == 0.0) && i != current_index + 1) {
    //   std::cout << i - 1 << std::endl;
    //   return std::vector<PlannerPoint>();
    // }
    auto next_point = router_points_[i];
    auto s = Distance(curr_point, next_point);
    auto target_yaw = Utils::GetYaw(router_points_[i], router_points_[i+1]);
    if (IsInJunction(i)) {
      auto speed_limit = constants::Constants::junction_speed;
      auto next_speed_and_yaw = NextSpeedAndYaw(curr_speed, speed_limit, curr_yaw, target_yaw, s);
      curr_point = next_point;
      curr_speed = next_speed_and_yaw.first;
      curr_yaw = next_speed_and_yaw.second;
    } else {
      int next_junction_index = NextCloseToJunction(i);
      if (next_junction_index >= 0) {
        double junction_speed_limit = constants::Constants::junction_speed;
        auto tmp_curr_point = curr_point;
        auto tmp_next_point = next_point;
        double tmp_s = 0.0;
        for (size_t j = i; j <= next_junction_index; j++) {
          tmp_s += Distance(tmp_curr_point, tmp_next_point);
          tmp_curr_point = tmp_next_point;
          tmp_next_point = router_points_[j+1];
        }
        double t = 2 * tmp_s / (curr_speed + junction_speed_limit);
        if (junction_speed_limit >= curr_speed) {
          double acce_limit = constants::Constants::acceleration;
          double a = std::min((junction_speed_limit - curr_speed) / t, acce_limit);
          double target_speed = std::sqrt(2 * a * s + curr_speed * curr_speed);
          auto next_speed_and_yaw = NextSpeedAndYaw(curr_speed, target_speed, curr_yaw, target_yaw, s);
          curr_point = next_point;
          curr_speed = next_speed_and_yaw.first;
          curr_yaw = next_speed_and_yaw.second;
        } else {
          double dece_limit = constants::Constants::deceleration;
          double a = std::min((curr_speed - junction_speed_limit) / t, dece_limit);
          double target_speed = std::sqrt(-2 * a * s + curr_speed * curr_speed);
          auto next_speed_and_yaw = NextSpeedAndYaw(curr_speed, target_speed, curr_yaw, target_yaw, s);
          curr_point = next_point;
          curr_speed = next_speed_and_yaw.first;
          curr_yaw = next_speed_and_yaw.second;
        }
      } else {
        // std::cout << "normal: " << i << std::endl;
        auto speed_limit = constants::Constants::speed;
        auto next_speed_and_yaw = NextSpeedAndYaw(curr_speed, speed_limit, curr_yaw, target_yaw, s);
        curr_point = next_point;
        curr_speed = next_speed_and_yaw.first;
        curr_yaw = next_speed_and_yaw.second;
      }
    }
    planner_points.emplace_back(next_point, curr_speed, curr_yaw);
  }

  return planner_points;
}

bool Planner::IsInJunction(size_t index) {
  for (const auto& pair : junction_points_pair_) {
    if (index >= pair.first && index < pair.second) {
      return true;
    }
  }
  return false;
}

int Planner::NextCloseToJunction(size_t index) {
  for (const auto& pair : junction_points_pair_) {
    if (pair.first <= index) {
      continue;
    }
    if (pair.first - index < look_ahead_index_) {
      return pair.first;
    } else {
      return -1;
    }
  }
  return -1;
}

std::pair<double, double> Planner::NextSpeedAndYaw(double current_speed, double target_speed, double current_yaw, double target_yaw, double s) {
  if (target_speed > current_speed) {
    double acce_limit = constants::Constants::acceleration;
    auto t_option = Utils::SolveFunction(0.5 * acce_limit, current_speed, -s);
    if (t_option == boost::none) {
      LOG_ERROR("no root. Something error when solve time function");
      return {0, 0};
    }
    auto t = std::max(t_option.value().first, t_option.value().second);
    if (t < 0) {
      LOG_ERROR("all values are less than 0. Something error when solve time function");
      return {0, 0};
    }

    double tmp_curr_speed = current_speed;
    current_speed += t * acce_limit;
    if (current_speed > target_speed) {
      current_speed = target_speed;
      double tmp_t = (target_speed - tmp_curr_speed) / acce_limit;
      double s_left = s - tmp_t * (tmp_curr_speed + target_speed) / 2.0;
      t = tmp_t + s_left / target_speed;
    }
    current_yaw = Utils::AfterRotate(current_yaw, target_yaw, t, constants::Constants::yaw_rate);
    return {current_speed, current_yaw};
  } else {
    double dece_limit = constants::Constants::deceleration;
    double t = (2 * s) / (current_speed + target_speed);
    double dece_real = (current_speed - target_speed) / t;
    if (dece_real > dece_limit) {
      LOG_WARNING("Deceleration is over limit, dece limit is %.2f, actual required dece is: %.2f", dece_limit, dece_real);
      auto t_option = Utils::SolveFunction(-0.5 * dece_limit, current_speed, -s);
      if (t_option == boost::none) {
        LOG_ERROR("no root. Something error when solve time function");
        return {0, 0};
      }
      t = std::max(t_option.value().first, t_option.value().second);
      if (t < 0) {
        LOG_ERROR("all values are less than 0. Something error when solve time function");
        return {0, 0};
      }
      current_speed -= t * dece_limit;
      current_yaw = Utils::AfterRotate(current_yaw, target_yaw, t, constants::Constants::yaw_rate);
      return {current_speed, current_yaw};
    } else {
      current_yaw = Utils::AfterRotate(current_yaw, target_yaw, t, constants::Constants::yaw_rate);
      return {target_speed, current_yaw};
    }


  }
}

// int main(int argc, char* argv[]) {
//   if (argc < 2) {
//     std::cout << "error, not enough params" << std::endl;
//     return 0;
//   }
//   std::string host(argv[1]);
//   uint16_t port(2000u);
//   auto client = carla::client::Client(host, port);
//   client.SetTimeout(10s);
//   auto world = client.GetWorld();
//   auto map = world.GetMap();
//   auto blueprint_library = world.GetBlueprintLibrary();
//   auto vehicles = blueprint_library->Filter("vehicle");
//   auto vehicle_bp = (*vehicles)[0];

//   auto transforms = (*map).GetRecommendedSpawnPoints();
//   auto size = transforms.size();

//   // SetAllTrafficLightToBeGreen(world);

//   size_t p1_i = 0;
//   size_t p2_i = size / 2;
//   if (argc >= 3) {
//     p1_i = std::stoi(argv[2]) % transforms.size();
//     if (argc >= 4) {
//       p2_i = std::stoi(argv[3]) % transforms.size();
//     }
//   }
//   Point3d p1(transforms[p1_i].location.x, transforms[p1_i].location.y,
//              transforms[p1_i].location.z);
//   Point3d p2(transforms[p2_i].location.x, transforms[p2_i].location.y,
//              transforms[p2_i].location.z);
//   std::cout << "start point: " << p1.ToString() << std::endl;
//   std::cout << "end point: " << p2.ToString() << std::endl;
//   auto vehicle1 = world.SpawnActor(vehicle_bp, transforms[p1_i]);
//   auto vehicle2 = world.SpawnActor(vehicle_bp, transforms[p2_i]);
//   vehicle1->SetSimulatePhysics(false);
//   auto init_yaw = vehicle1->GetTransform().rotation.yaw;
//   Planner planner(p1, p2, map, init_yaw, 0.0);

//   auto router_points = planner.GetRouterPoints();
//   p1 = router_points[0];

//   Odom init_odom(p1, 0.0, init_yaw);
//   auto points = planner.GetPlannerPoints(init_odom);
//   auto router_point = points[2].GetPoint();
//   auto current_speed = points[2].GetSpeed();
//   carla::geom::Rotation rotation1(0.0, points[2].GetYaw(), 0.0);
//   carla::geom::Transform transfrom1(
//       carla::geom::Location(router_point.x_, router_point.y_,
//                             router_point.z_),
//       rotation1);
//   vehicle1->SetTransform(transfrom1);
//   std::this_thread::sleep_for(100ms);

//   while (points.size() != 0) {
//     auto location = vehicle1->GetLocation();
//     auto current_yaw = vehicle1->GetTransform().rotation.yaw;
//     auto current_pos = Point3d(location.x, location.y, 0);
//     Odom current_odom(current_pos, current_speed, current_yaw);
//     auto next_pos = planner.GetPlannerPoints(current_odom);
//     std::cout << "next pos size: " << next_pos.size() << std::endl;
//     if (next_pos.size() <= 1) {
//       vehicle1->Destroy();
//       vehicle2->Destroy();
//       return 0;
//     }
//     router_point = next_pos[1].GetPoint();
//     std::cout << "current pos: " << current_pos.ToString() << std::endl;
//     std::cout << "next pos: " << router_point.ToString() << std::endl;
//     carla::geom::Rotation rotation(0.0, next_pos[0].GetYaw(), 0.0);
//     carla::geom::Transform transfrom(
//         carla::geom::Location(router_point.x_, router_point.y_,
//                               router_point.z_),
//         rotation);
//     vehicle1->SetTransform(transfrom);
//     world.WaitForTick(100ms);
//     std::this_thread::sleep_for(50ms);
//     // points = planner.GetPlannerPoints(Odom(Point3d(location.x, location.y, location.z), points[0].GetSpeed(), points[0].GetYaw()));

//   }



//   std::cin.ignore();
//   std::cin.get();
//   vehicle1->Destroy();
//   vehicle2->Destroy();
//   return 0;
// }
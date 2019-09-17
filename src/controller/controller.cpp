/*
 * File: controller.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 16th September 2019 7:56:40 pm
 */

#include "controller/pid_controller.h"
#include <memory>

using namespace minjun;
using namespace minjun::utils;
using namespace std::chrono_literals;
using namespace std::string_literals;

// int main() {
// }
int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "error, not enough params" << std::endl;
    return 0;
  }
  double fps = 30.0;
  std::string host(argv[1]);
  uint16_t port(2000u);
  auto client = carla::client::Client(host, port);
  client.SetTimeout(10s);
  auto world = client.GetWorld();
  auto settings = world.GetSettings();
  auto previous_settings = settings;
  settings.synchronous_mode = true;
  settings.fixed_delta_seconds = 1.0 / fps;
  world.ApplySettings(settings);
  auto map = world.GetMap();
  auto blueprint_library = world.GetBlueprintLibrary();
  auto vehicles = blueprint_library->Filter("vehicle");
  auto vehicle_bp = (*vehicles)[0];

  auto transforms = (*map).GetRecommendedSpawnPoints();
  auto size = transforms.size();

  // SetAllTrafficLightToBeGreen(world);

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
  // vehicle1->SetSimulatePhysics(false);
  auto init_yaw = vehicle1->GetTransform().rotation.yaw;
  Planner planner(p1, p2, map, init_yaw, 0.0);
  std::shared_ptr<Controller> controller = std::make_shared<PIDController>(0.2, 0.05, 0.01);

  auto router_points = planner.GetRouterPoints();
  p1 = router_points[0];

  Odom init_odom(p1, 0.0, init_yaw);
  auto points = planner.GetPlannerPoints(init_odom);
  auto router_point = points[2].GetPoint();
  auto current_speed = points[2].GetSpeed();
  carla::geom::Rotation rotation1(0.0, points[2].GetYaw(), 0.0);
  carla::geom::Transform transfrom1(
      carla::geom::Location(router_point.x_, router_point.y_,
                            router_point.z_),
      rotation1);
  vehicle1->SetTransform(transfrom1);
  std::this_thread::sleep_for(100ms);

  while (points.size() != 0) {
    world.Tick();
    auto location = vehicle1->GetLocation();
    auto current_yaw = vehicle1->GetTransform().rotation.yaw;
    auto current_pos = Point3d(location.x, location.y, 0);
    current_speed = Vector3DToDoulbe(boost::static_pointer_cast<carla::client::Vehicle>(vehicle1)->GetVelocity());
    Odom current_odom(current_pos, current_speed, current_yaw);
    auto next_pos = planner.GetPlannerPoints(current_odom);
    // std::cout << "next pos size: " << next_pos.size() << std::endl;
    if (next_pos.size() <= 4) {
      vehicle1->Destroy();
      vehicle2->Destroy();
      world.ApplySettings(previous_settings);
      return 0;
    }
    router_point = next_pos[1].GetPoint();
    std::cout << "current pos: " << current_pos.ToString() << std::endl;
    std::cout << "target pos: " << router_point.ToString() << std::endl;
    auto control = controller->Control(current_odom, next_pos);
    std::cout << "throttle: " << control.throttle << std::endl;
    std::cout << "steer: " << control.steer << std::endl;
    carla::client::Vehicle::Control carla_control;
    carla_control.throttle = control.throttle;
    carla_control.steer = control.steer;
    carla_control.brake = control.brake;
    boost::static_pointer_cast<carla::client::Vehicle>(vehicle1)->ApplyControl(carla_control);
    // carla::geom::Rotation rotation(0.0, next_pos[0].GetYaw(), 0.0);
    // carla::geom::Transform transfrom(
    //     carla::geom::Location(router_point.x_, router_point.y_,
    //                           router_point.z_),
    //     rotation);
    // vehicle1->SetTransform(transfrom);
    // world.WaitForTick(100ms);
    // std::this_thread::sleep_for(30ms);
    // points = planner.GetPlannerPoints(Odom(Point3d(location.x, location.y, location.z), points[0].GetSpeed(), points[0].GetYaw()));

  }



  std::cin.ignore();
  std::cin.get();
  vehicle1->Destroy();
  vehicle2->Destroy();
  world.ApplySettings(previous_settings);
  return 0;
}
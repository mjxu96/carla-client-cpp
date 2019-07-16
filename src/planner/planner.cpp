#include "planner/planner.h"

using namespace minjun;
using namespace minjun::utils;
using namespace minjun::planner;

using namespace std::chrono_literals;
using namespace std::string_literals;

Planner::Planner(Point3d start_point, Point3d end_point) :
  start_point_(std::move(start_point)), end_point_(std::move(end_point)) {}
std::vector<PlannerPoint> Planner::GetPlannerPoints() {

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
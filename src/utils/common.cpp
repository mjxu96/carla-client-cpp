/*
 * File: common.cpp
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 16th July 2019 9:08:17 pm
 */

#include "utils/common.h"


using namespace minjun::utils;

bool Point3d::operator==(const Point3d& p) const {
  return std::sqrt(std::pow((p.x_ - x_), 2.0) + std::pow((p.y_ - y_), 2.0) +
                   std::pow((p.z_ - z_), 2.0)) < 0.001;
}
std::string Point3d::ToString() const {
  return std::string("[") + std::to_string(x_) + std::string(", ") +
         std::to_string(y_) + std::string(", ") + std::to_string(z_) +
         std::string("]");
}


MapUtils::MapUtils(boost::shared_ptr<carla::client::Map> map_ptr) :
  map_ptr_(std::move(map_ptr)) {}


namespace std {
template <>
struct hash<minjun::utils::Point3d> {
  size_t operator()(const minjun::utils::Point3d& p) const {
    return (hash<double>()(p.x_)) ^ (hash<double>()(p.y_)) ^
           (hash<double>()(p.z_));
  }
};
}  // namespace std
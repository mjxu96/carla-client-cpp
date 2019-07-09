/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 8th July 2019 9:10:02 pm
 */

#ifndef MINJUN_ROUTER_COMMON_H_
#define MINJUN_ROUTER_COMMON_H_

#include <boost/geometry.hpp>
#include <functional>
#include <string>

namespace minjun {

class Point3d {
 public:
  Point3d(double x, double y, double z) : x_(x), y_(y), z_(z) {}
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  bool operator==(const Point3d& p) const {
    return x_ == p.x_ && y_ == p.y_ && z_ == p.z_;
  }

  std::string ToString() {
    return std::string("[") + std::to_string(x_) + std::string(", ") +
                              std::to_string(y_) + std::string(", ") +
                              std::to_string(z_) + std::string("]");
  }

};

}  // namespace minjun

namespace std {
  template <>
  struct hash<minjun::Point3d> {
    size_t operator() (const minjun::Point3d& p) const {
      return (hash<double>()(p.x_)) ^ (hash<double>()(p.y_)) ^ (hash<double>()(p.z_));
    }
  };
}
#endif
/*
 * File: common.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Tuesday, 16th July 2019 9:06:00 pm
 */

#ifndef MINJUN_COMMON_H_
#define MINJUN_COMMON_H_

#include "carla/client/Map.h"

#include <boost/shared_ptr.hpp>

namespace minjun {

namespace utils {

class Point3d {
 public:
  Point3d() = default;
  Point3d(float x, float y, float z) : x_(x), y_(y), z_(z) {}
  float x_{0.0};
  float y_{0.0};
  float z_{0.0};

  bool operator==(const Point3d& p) const;


  std::string ToString() const;
};

class MapUtils {
public:
  MapUtils(boost::shared_ptr<carla::client::Map> map_ptr);


private:
  boost::shared_ptr<carla::client::Map> map_ptr_{nullptr};

};
  
} // namespace utils 
  
} // namespace minjun



#endif
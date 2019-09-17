/*
 * File: constants.h
 * Author: Minjun Xu (mjxu96@gmail.com)
 * File Created: Monday, 15th July 2019 9:59:21 pm
 */

#ifndef MINJUN_UTILS_CONSTANTS_H_
#define MINJUN_UTILS_CONSTANTS_H_

namespace minjun {
namespace constants {

class Constants {
 public:
  static constexpr double acceleration = 2.5;       // m/s^2
  static constexpr double deceleration = 7.5;       // m/s^2
  static constexpr double speed = 15.0;             // m/s
  static constexpr double junction_speed = 5.0;     // m/s
  static constexpr double yaw_rate = 50.0;          // degree
};

}  // namespace constants
}  // namespace minjun

#endif
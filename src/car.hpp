//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_CAR_HPP
#define PATH_PLANNING_CAR_HPP

#include "fsm.hpp"
#include "helpers.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>

struct Car {
  int    id;
  State  state;
  double x_m;
  double y_m;
  double s_m;
  double d_m;
  double vel_mps;
  double vel_x_mps;
  double vel_y_mps;
  double yaw_rad;
  double acc_s_mps2;
  double acc_d_mps2;

  static Car FromVector(const std::vector<double>& car_info);
};

inline std::ostream& operator<<(std::ostream& ostream, const Car& car) {
  ostream << std::fixed
          << "Car{\n"
          << "  .id         = " << car.id         << ",\n"
          << "  .state      = " << car.state      << ",\n"
          << "  .x_m        = " << car.x_m        << ",\n"
          << "  .y_m        = " << car.y_m        << ",\n"
          << "  .s_m        = " << car.s_m        << ",\n"
          << "  .d_m        = " << car.d_m        << ",\n"
          << "  .vel_mps    = " << car.vel_mps    << ",\n"
          << "  .vel_x_mps  = " << car.vel_x_mps  << ",\n"
          << "  .vel_y_mps  = " << car.vel_y_mps  << ",\n"
          << "  .yaw_rad    = " << car.yaw_rad    << ",\n"
          << "  .acc_s_mps2 = " << car.acc_s_mps2 << ",\n"
          << "  .acc_d_mps2 = " << car.acc_d_mps2 << ",\n"
          << "}";

  return ostream;
}

inline bool operator==(const Car& car1, const Car& car2) {
  return not static_cast<bool>(std::memcmp(&car1, &car2, sizeof(Car)));
}

inline bool operator<(const Car& car1, const Car& car2) {
  return (car1.id < car2.id);
}

#endif //PATH_PLANNING_CAR_HPP

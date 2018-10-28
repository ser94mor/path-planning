//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_CAR_HPP
#define PATH_PLANNING_CAR_HPP

#include "state.hpp"

#include <iostream>

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
};

inline std::ostream& operator<<(std::ostream& ostream, const Car& car) {
  ostream << std::fixed
          << "Car{\n"
          << "  .id        = " << car.id        << ",\n"
          << "  .state     = " << car.state     << ",\n"
          << "  .x_m       = " << car.x_m       << ",\n"
          << "  .y_m       = " << car.y_m       << ",\n"
          << "  .s_m       = " << car.s_m       << ",\n"
          << "  .d_m       = " << car.d_m       << ",\n"
          << "  .vel_mps   = " << car.vel_mps   << ",\n"
          << "  .vel_x_mps = " << car.vel_x_mps << ",\n"
          << "  .vel_y_mps = " << car.vel_y_mps << ",\n"
          << "  .yaw_rad   = " << car.yaw_rad   << ",\n"
          << "}";

  return ostream;
}

#endif //PATH_PLANNING_CAR_HPP

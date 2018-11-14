//
// Created by aoool on 14.11.18.
//

#include "car.hpp"

Car Car::FromVector(const std::vector<double>& car_info) {
  return {
      .id        = static_cast<int>(car_info[0]),
      .state     = State::KeepLane,
      .x_m       = car_info[1],
      .y_m       = car_info[2],
      .s_m       = car_info[5],
      .d_m       = car_info[6],
      .vel_mps   = sqrt(car_info[3]*car_info[3] + car_info[4]*car_info[4]),
      .vel_x_mps = car_info[3],
      .vel_y_mps = car_info[4],
      .yaw_rad   = calc_yaw_rad(car_info[3], car_info[4]),
  };
}

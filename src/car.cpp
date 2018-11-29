//
// Created by aoool on 14.11.18.
//

#include "car.hpp"

Car Car::FromVector(const std::vector<double>& car_info, const PathPlannerConfig& config) {
  double vel = sqrt(car_info[3]*car_info[3] + car_info[4]*car_info[4]);
  double yaw = CalcYawRad(car_info[3], car_info[4]);
  double x = car_info[1];
  double y = car_info[2];
  double vel_x = car_info[3];
  double vel_y = car_info[4];
  double s = car_info[5];
  double d = car_info[6];
  auto frenet_vel = GetFrenetSpeed(s, d, x, y, vel_x, vel_y, config);
  double vel_s = frenet_vel[0];
  double vel_d = frenet_vel[1];

  return {
      .id         = static_cast<int>(car_info[0]),
      .state      = State::KeepLane,
      .vel_mps    = vel,
      .yaw_rad    = yaw,
      .x_m        = x,
      .y_m        = y,
      .vel_x_mps  = vel_x,
      .vel_y_mps  = vel_y,
      .acc_x_mps2 = 0.0,
      .acc_y_mps2 = 0.0,
      .s_m        = s,
      .d_m        = d,
      .vel_s_mps  = vel_s,
      .vel_d_mps  = vel_d,
      .acc_s_mps2 = 0.0,
      .acc_d_mps2 = 0.0,
  };
}

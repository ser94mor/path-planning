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

struct CartesianCar;

struct FrenetCar {
  int id;
  State state;
  double vel_mps;

  double s_m;
  double d_m;
  double vel_s_mps;
  double vel_d_mps;
  double acc_s_mps2;
  double acc_d_mps2;

  double GetVelocity() const;

  /**
 * @brief Transforms data provided by one item from sensor fusion vector to FrenetCar.
 * @throw std::domain_error If the car does not keep lane or moves with the acceleration.
 * @return Instance of FrenetCar corresponding to this Cartesian car.
 */
  static FrenetCar
  FromVectorAssumingConstantVelocityAndLaneKeeping(const std::vector<double>& car_info,
                                                   const PathPlannerConfig& config);
};

struct CartesianCar {
  int    id;
  State  state;

  double vel_mps;

  double x_m;
  double y_m;
  double vel_x_mps;
  double vel_y_mps;
  double acc_x_mps2;
  double acc_y_mps2;

  double GetVelocity() const;
  double GetYaw() const;
};

struct Car {
  int    id;
  State  state;

  double vel_mps;
  double yaw_rad;

  double x_m;
  double y_m;
  double vel_x_mps;
  double vel_y_mps;
  double acc_x_mps2;
  double acc_y_mps2;

  double s_m;
  double d_m;
  double vel_s_mps;
  double vel_d_mps;
  double acc_s_mps2;
  double acc_d_mps2;

  static Car FromVector(const std::vector<double>& car_info, const PathPlannerConfig& config);
};

inline std::ostream& operator<<(std::ostream& ostream, const Car& car) {
  ostream << std::fixed
          << "Car{\n"
          << "  .id         = " << car.id         << ",\n"
          << "  .state      = " << car.state      << ",\n"
          << "  .vel_mps    = " << car.vel_mps    << ",\n"
          << "  .yaw_rad    = " << car.yaw_rad    << ",\n"
          << "  .x_m        = " << car.x_m        << ",\n"
          << "  .y_m        = " << car.y_m        << ",\n"
          << "  .vel_x_mps  = " << car.vel_x_mps  << ",\n"
          << "  .vel_y_mps  = " << car.vel_y_mps  << ",\n"
          << "  .acc_x_mps2 = " << car.acc_x_mps2 << ",\n"
          << "  .acc_y_mps2 = " << car.acc_y_mps2 << ",\n"
          << "  .s_m        = " << car.s_m        << ",\n"
          << "  .d_m        = " << car.d_m        << ",\n"
          << "  .vel_s_mps  = " << car.vel_s_mps  << ",\n"
          << "  .vel_d_mps  = " << car.vel_d_mps  << ",\n"
          << "  .acc_s_mps2 = " << car.acc_s_mps2 << ",\n"
          << "  .acc_d_mps2 = " << car.acc_d_mps2 << ",\n"
          << "}";

  return ostream;
}


inline bool operator==(const Car& car1, const Car& car2)
{
  return not static_cast<bool>(std::memcmp(&car1, &car2, sizeof(Car)));
}


inline bool operator==(const FrenetCar& car1, const FrenetCar& car2)
{
  return not static_cast<bool>(std::memcmp(&car1, &car2, sizeof(FrenetCar)));
}


inline bool operator<(const Car& car1, const Car& car2)
{
  return (car1.id < car2.id);
}


inline bool operator<(const FrenetCar& car1, const FrenetCar& car2)
{
  return (car1.id < car2.id);
}

#endif //PATH_PLANNING_CAR_HPP

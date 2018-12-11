//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_CAR_HPP
#define PATH_PLANNING_CAR_HPP

#include "fsm.hpp"
#include "helpers.hpp"
#include "circular_unsigned_double_t.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <optional>
#include <memory>



struct FrenetCar {
  int id;
  State state;
  double vel_mps;
  double time_s;

  circular_unsigned_double_t s_m;
  double d_m;
  double vel_s_mps;
  double vel_d_mps;
  double acc_s_mps2;
  double acc_d_mps2;

  double LongitudinalForwardDistanceTo(const FrenetCar& car);
  double LongitudinalBackwardDistanceTo(const FrenetCar& car);
  double LateralDistanceTo(const FrenetCar& car);

  bool IsFrontBufferViolatedBy(const FrenetCar& car);
  bool IsBackBufferViolatedBy(const FrenetCar& car);
  bool IsSideBufferViolatedBy(const FrenetCar& car);

  /**
 * @brief Transforms data provided by one item from sensor fusion vector to FrenetCar.
 * @throw std::domain_error If the car does not keep lane or moves with the acceleration.
 * @return Instance of FrenetCar corresponding to this Cartesian car.
 */
  static FrenetCar
  FromVectorAssumingConstantVelocityAndLaneKeeping(const std::vector<double>& car_info, double time,
                                                   const PathPlannerConfig& config);
  static void SetPathPlannerConfig(const PathPlannerConfig* pp_config);

  static const PathPlannerConfig* pp_config_;
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


inline std::ostream& operator<<(std::ostream& ostream, const FrenetCar& car)
{
  ostream << std::fixed
          << "FrenetCar{\n"
          << "  .id         = " << car.id         << ",\n"
          << "  .state      = " << car.state      << ",\n"
          << "  .vel_mps    = " << car.vel_mps    << ",\n"
          << "  .time_s     = " << car.time_s     << ",\n"
          << "  .s_m        = " << car.s_m        << ",\n"
          << "  .d_m        = " << car.d_m        << ",\n"
          << "  .vel_s_mps  = " << car.vel_s_mps  << ",\n"
          << "  .vel_d_mps  = " << car.vel_d_mps  << ",\n"
          << "  .acc_s_mps2 = " << car.acc_s_mps2 << ",\n"
          << "  .acc_d_mps2 = " << car.acc_d_mps2 << ",\n"
          << "}";

  return ostream;
}


inline bool operator==(const FrenetCar& car1, const FrenetCar& car2)
{
  return car1.id == car2.id                        &&
         car1.state == car2.state                  &&
         IsEqual(car1.vel_mps, car2.vel_mps)       &&
         IsEqual(car1.time_s, car2.time_s)         &&
         car1.s_m == car2.s_m                      &&
         IsEqual(car1.d_m, car2.d_m)               &&
         IsEqual(car1.vel_s_mps, car2.vel_s_mps)   &&
         IsEqual(car1.vel_d_mps, car2.vel_d_mps)   &&
         IsEqual(car1.acc_s_mps2, car2.acc_s_mps2) &&
         IsEqual(car1.acc_d_mps2, car2.acc_d_mps2);
}


inline bool operator<(const Car& car1, const Car& car2)
{
  return (car1.id < car2.id);
}


inline bool operator<(const FrenetCar& car1, const FrenetCar& car2)
{
  return (car1.id < car2.id);
}

inline std::vector<FrenetCar> GetFrenetCarsInLane(int lane, double lane_width, const std::vector<FrenetCar>& all_cars)
{
  std::vector<FrenetCar> cars_in_lane;
  for (const auto& car : all_cars) {
    if (CalcLaneNumber(car.d_m, lane_width) == lane) {
      cars_in_lane.push_back(car);
    }
  }

  return cars_in_lane;
}

inline std::optional<FrenetCar>
GetNearestFrenetCarAheadBySCoordIfPresent(const FrenetCar& ego_car, const std::vector<FrenetCar> cars)
{
  std::optional<FrenetCar> opt{std::nullopt};

  for (const auto& car : cars) {
    if (car.s_m >= ego_car.s_m && (!opt.has_value() || opt.value().s_m > car.s_m)) {
      opt = car;
    }
  }

  return opt;
}

#endif //PATH_PLANNING_CAR_HPP

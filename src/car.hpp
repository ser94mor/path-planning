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


struct Car {
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

  double LongitudinalForwardDistanceTo(const Car& car) const;
  double LongitudinalBackwardDistanceTo(const Car& car) const;
  double LateralDistanceTo(const Car& car) const;

  bool IsFrontBufferViolatedBy(const Car& car) const;
  bool IsBackBufferViolatedBy(const Car& car) const;
  bool IsSideBufferViolatedBy(const Car& car) const;

  bool IsInFrontOf(const Car& car) const;
  bool IsBehind(const Car& car) const;

  std::vector<Car> CarsInRegionOfInterest(const std::vector<Car>& cars) const;

  int Lane() const;

  /**
 * @brief Transforms data provided by one item from sensor fusion vector to Car.
 * @throw std::domain_error If the car does not keep lane or moves with the acceleration.
 * @return Instance of Car corresponding to this Cartesian car.
 */
  static Car
  FromVectorAssumingConstantVelocityAndLaneKeeping(const std::vector<double>& car_info, double time,
                                                   const PathPlannerConfig& config);
  static void SetPathPlannerConfig(const PathPlannerConfig* pp_config);

  static const PathPlannerConfig* pp_config_;
};


inline std::ostream& operator<<(std::ostream& ostream, const Car& car)
{
  ostream << std::fixed
          << "Car{\n"
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


inline bool operator==(const Car& car1, const Car& car2)
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

inline std::vector<Car> GetCarsInLane(int lane, double lane_width, const std::vector<Car>& all_cars)
{
  std::vector<Car> cars_in_lane;
  for (const auto& car : all_cars) {
    if (car.Lane() == lane) {
      cars_in_lane.push_back(car);
    }
  }

  return cars_in_lane;
}

inline std::optional<Car>
GetNearestCarAheadBySCoordIfPresent(const Car& ego_car, const std::vector<Car> cars)
{
  std::optional<Car> opt{std::nullopt};

  for (const auto& car : cars) {
    if (car.s_m >= ego_car.s_m && (!opt.has_value() || opt.value().s_m > car.s_m)) {
      opt = car;
    }
  }

  return opt;
}

#endif //PATH_PLANNING_CAR_HPP

//
// Created by aoool on 18.12.18.
//

#ifndef PATH_PLANNING_COST_FUNCTIONS_HPP
#define PATH_PLANNING_COST_FUNCTIONS_HPP

#include "helpers.hpp"
#include "car.hpp"


inline double CarAheadCost(const Car& planned_ego_car, const std::map<Car, Car>& predictions)
{
  auto nearest_car_ahead_opt = planned_ego_car.NearestCarAheadInIntendedLane(map_vals(predictions));
  if (nearest_car_ahead_opt.has_value()) {
    return Logistic(planned_ego_car.LongitudinalForwardDistanceTo(nearest_car_ahead_opt.value()));
  } else {
    return 0.0;

  }
}


inline double LaneMaxSpeedCost(const Car& planned_ego_car, const std::map<Car, Car>& predictions)
{
  auto intended_lane_nearest_car_ahead_opt = planned_ego_car.NearestCarAheadInIntendedLane(map_vals(predictions));

  double intended_lane_velocity = planned_ego_car.IsFrontBufferViolatedBy(*intended_lane_nearest_car_ahead_opt, 2.0) ?
                                  intended_lane_nearest_car_ahead_opt->Vs() : Car::MaxVelocity();

  if (intended_lane_velocity >= Car::MaxVelocity()) {
    return 0.0;
  } else {
    return (Car::MaxVelocity() - intended_lane_velocity) / Car::MaxVelocity();
  }
}


inline double LaneCost(const Car& planned_ego_car, const std::map<Car, Car>& predictions)
{
  if (planned_ego_car.IsInRightMostLane()) {
    return 1.0;
  } else {
    return 0.0;
  }
}


#endif //PATH_PLANNING_COST_FUNCTIONS_HPP

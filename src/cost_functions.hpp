//
// Created by aoool on 18.12.18.
//

#ifndef PATH_PLANNING_COST_FUNCTIONS_HPP
#define PATH_PLANNING_COST_FUNCTIONS_HPP

#include "helpers.hpp"
#include "car.hpp"


inline double CarAheadCost(const Car& cur_ego_car, const Car& planned_ego_car, const std::map<Car, Car>& predictions)
{
  auto nearest_car_ahead_opt = planned_ego_car.NearestCarAheadInFinalLane(map_vals(predictions));
  if (nearest_car_ahead_opt.has_value()) {
    return Logistic(planned_ego_car.LongitudinalForwardDistanceTo(nearest_car_ahead_opt.value()));
  } else {
    return 0.0;

  }
}


inline double LaneMaxSpeedCost(const Car& cur_ego_car, const Car& planned_ego_car, const std::map<Car, Car>& predictions)
{
  auto intended_lane_nearest_car_ahead_opt = planned_ego_car.NearestCarAheadInIntendedLane(map_vals(predictions));
  auto current_lane_nearest_car_ahead_opt  = planned_ego_car.NearestCarAheadInCurrentLane(map_vals(predictions));

  double intended_lane_velocity = planned_ego_car.IsFrontBufferViolatedBy(*intended_lane_nearest_car_ahead_opt, 3.0) ?
                                  intended_lane_nearest_car_ahead_opt->Vs() : Car::MaxVelocity();
  double current_lane_velocity  = planned_ego_car.IsFrontBufferViolatedBy(*current_lane_nearest_car_ahead_opt, 3.0) ?
                                  current_lane_nearest_car_ahead_opt->Vs() : Car::MaxVelocity();

  if (intended_lane_velocity >= Car::MaxVelocity()) {
    return 0.0;
  } else {
    return (Car::MaxVelocity() - intended_lane_velocity) / Car::MaxVelocity();
  }
}


inline double BufferViolationCost(const Car& cur_ego_car, const Car& planned_ego_car, const std::map<Car, Car>& predictions)
{
  auto planned_nearest_car_ahead_in_final_lane_opt = planned_ego_car.NearestCarAheadInFinalLane(map_vals(predictions));
  auto planned_nearest_car_behind_in_final_lane_opt = planned_ego_car.NearestCarBehindInFinalLane(map_vals(predictions));

  if ( (planned_nearest_car_ahead_in_final_lane_opt.has_value() &&
       planned_ego_car.IsFrontBufferViolatedBy(planned_nearest_car_ahead_in_final_lane_opt.value())) ||
       (planned_nearest_car_behind_in_final_lane_opt.has_value() &&
       planned_ego_car.IsBackBufferViolatedBy(planned_nearest_car_behind_in_final_lane_opt.value()))) {
    return 1.0;
  }

  return 0.0;
}

#endif //PATH_PLANNING_COST_FUNCTIONS_HPP

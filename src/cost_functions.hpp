//
// Created by aoool on 18.12.18.
//

#ifndef PATH_PLANNING_COST_FUNCTIONS_HPP
#define PATH_PLANNING_COST_FUNCTIONS_HPP

#include "helpers.hpp"
#include "car.hpp"


inline double CarAheadCost(const Car& ego_car, const std::vector<Car>& other_cars)
{
  auto&& in_intr_reg_other_cars = ego_car.CarsInRegionOfInterest(other_cars);
  auto&& cur_lane_other_cars = ego_car.CarsInCurrentLane(in_intr_reg_other_cars);

  auto nearest_car_ahead_opt = ego_car.NearestCarAhead(cur_lane_other_cars);
  if (nearest_car_ahead_opt.has_value()) {
    return Logistic(ego_car.LongitudinalForwardDistanceTo(nearest_car_ahead_opt.value()));
  } else {
    return 0.0;
  }
}


inline double LaneMaxSpeedCost(const Car& ego_car, const std::vector<Car>& other_cars)
{
  auto&& in_intr_reg_other_cars = ego_car.CarsInRegionOfInterest(other_cars);
  auto&& intended_lane_other_cars = ego_car.CarsInIntendedLane(in_intr_reg_other_cars);

  auto nearest_car_ahead_opt = ego_car.NearestCarAhead(intended_lane_other_cars);

  if (nearest_car_ahead_opt.has_value() && ego_car.IsFrontBufferViolatedBy(nearest_car_ahead_opt.value(), 3.0)) {
    return (Car::MaxVelocity() - nearest_car_ahead_opt.value().Vs()) / Car::MaxVelocity();
  } else {
    return 0.0;
  }
}


inline double BufferViolationCost(const Car& ego_car, const std::vector<Car>& other_cars)
{
  auto nearest_car_ahead_in_final_lane_opt = ego_car.NearestCarAheadInFinalLane(other_cars);
  auto nearest_car_behind_in_final_lane_opt = ego_car.NearestCarBehindInFinalLane(other_cars);

  if ( (nearest_car_ahead_in_final_lane_opt.has_value() &&
        ego_car.IsFrontBufferViolatedBy(nearest_car_ahead_in_final_lane_opt.value(), 0.9)) ||
       (nearest_car_behind_in_final_lane_opt.has_value() &&
        ego_car.IsBackBufferViolatedBy(nearest_car_behind_in_final_lane_opt.value(), 0.9))) {
    return 1.0;
  }

  return 0.0;
}

#endif //PATH_PLANNING_COST_FUNCTIONS_HPP

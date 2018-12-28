//
// Created by aoool on 28.10.18.
//

#include "behavior_layer.hpp"
#include "cost_functions.hpp"

#include <typeinfo>
#include <cassert>
#include <algorithm>
#include <iostream>


BehaviorLayer::BehaviorLayer(const PathPlannerConfig& path_planner_config,
                             LocalizationLayer& localization_layer,
                             PredictionLayer& prediction_layer):
  pp_config_{path_planner_config},
  localization_layer_{localization_layer},
  prediction_layer_{prediction_layer}
{

}


std::vector<Car> BehaviorLayer::Plan(const Car& ego_car) const
{
  const auto&& predictions = prediction_layer_.GetPredictions(pp_config_.behavior_planning_time_horizon_s, ego_car.T());

  std::vector<Car> planned_ego_cars;
  for (auto state : ego_car.PossibleNextStates()) {
    Car cur_ego_car = Car::Builder(ego_car).SetState(state).Build();

    std::cout << __PRETTY_FUNCTION__ << " started planning for ego car\n" << cur_ego_car << std::endl;

    planned_ego_cars.push_back(PlanForState(cur_ego_car, predictions));

    std::cout << __PRETTY_FUNCTION__ << " finished planning; the planned car for state " << state << " is\n"
              << planned_ego_cars[planned_ego_cars.size() - 1] << std::endl;
  }

  std::map<Car, Car> cars_considered;
  for (const auto& car: ego_car.CarsInRegionOfInterest(map_keys(predictions))) {
    cars_considered.insert(std::make_pair(car, predictions.at(car)));
  }
  std::cout << __PRETTY_FUNCTION__ << " cars taken into consideration are:\n"
            << Car::CarMapToString(cars_considered) << std::endl;
  
  assert(!planned_ego_cars.empty());

  SortPlannedEgoCarsByPriority(planned_ego_cars, ego_car, predictions);

  return planned_ego_cars;
}


double BehaviorLayer::PlannedEgoCarCost(const Car& cur_ego_car,
                                        const Car& planned_ego_car,
                                        const std::map<Car, Car>& predictions) const
{
  return 0.0001 * LaneCost(cur_ego_car, planned_ego_car, predictions) +    // penalize for being in certain lane
         0.5 * CarAheadCost(cur_ego_car, planned_ego_car, predictions) +
         LaneMaxSpeedCost(cur_ego_car,planned_ego_car, predictions) +
         0.5 * ProgressCost(cur_ego_car, planned_ego_car, predictions);
}


void BehaviorLayer::SortPlannedEgoCarsByPriority(std::vector<Car>& planned_ego_cars, const Car& cur_ego_car,
                                                 const std::map<Car, Car>& predictions) const
{
  // sort in accordance with the cost of the planned car in ascending order
  std::stable_sort(planned_ego_cars.begin(), planned_ego_cars.end(),
                   [&predictions, this, &cur_ego_car](const auto& car1, const auto& car2) {
    return PlannedEgoCarCost(cur_ego_car, car1, predictions) < PlannedEgoCarCost(cur_ego_car, car2, predictions);
  });

  for (const auto& planned_car : planned_ego_cars) {
    std::cout << __PRETTY_FUNCTION__ << " cost of the " << planned_car.State() << " state is "
              << PlannedEgoCarCost(cur_ego_car, planned_car, predictions) << '.' << std::endl;
  }
}


Car BehaviorLayer::PlanForState(const Car& ego_car, const std::map<Car, Car>& predictions) const
{
  // first, assume that there are no obstacles and plan in accordance with this assumption
  Car planned_ego_car_no_obstacles = PlanWithNoObstacles(ego_car, pp_config_.behavior_planning_time_horizon_s);
  std::optional<Car> planned_ego_car_with_obstacles =
      PlanWithObstacles(ego_car, predictions, pp_config_.behavior_planning_time_horizon_s);

  if (planned_ego_car_with_obstacles && planned_ego_car_with_obstacles->S() <= planned_ego_car_no_obstacles.S()) {
    return *planned_ego_car_with_obstacles;
  } else {
    return planned_ego_car_no_obstacles;
  }
}


Car BehaviorLayer::PlanWithNoObstacles(const Car& ego_car, double t) const
{
  double s_max_speed_at_t = fmin(Calc1DVelocity(ego_car.Vs(), ego_car.As(), pp_config_.max_jerk_mps3, t),
                                 pp_config_.max_speed_mps);

  double s = Calc1DPosition(static_cast<double>(ego_car.S()), (s_max_speed_at_t + ego_car.Vs()) / 2.0, 0.0, t);

  double d = (ego_car.FinalLane() + 0.5) * pp_config_.lane_width_m; // + 0.5 means lane center

  return Car::Builder(ego_car)
           .SetTime(ego_car.T() + t)
           .SetCoordinateS(s)
           .SetCoordinateD(d)
           .SetVelocityS(s_max_speed_at_t)
           .SetVelocityD(0.0)
           .SetAccelerationS(0.0)
           .SetAccelerationD(0.0)
         .Build();
}


std::optional<Car>
BehaviorLayer::PlanWithObstacles(const Car& ego_car, const std::map<Car, Car>& predictions, double t) const {
  auto&& all_cur_cars = map_keys(predictions);

  std::optional<Car> intended_lane_cur_car_ahead_opt = ego_car.NearestCarAheadInIntendedLane(all_cur_cars);
  std::optional<Car> final_lane_other_car_ahead_opt = ego_car.NearestCarAheadInFinalLane(all_cur_cars);

  std::optional<Car> intended_lane_cur_car_behind_opt = ego_car.NearestCarBehindInIntendedLane(all_cur_cars);
  
  std::optional<Car> future_car_to_follow{std::nullopt};

  // The worst possible position is when the nearest car behind ego vehicle violates the safety buffer.
  // In this case we choose to go behind that car because we risk causing an accident during the lane change otherwise.
  // This is not applicable to the lane keeping, because in this case the car behind is responsible for 
  // maintaining a safe distance.
  if (intended_lane_cur_car_behind_opt && ego_car.State() != FSM::State::KeepLane) {
    if (ego_car.IsBackBufferViolatedBy(*intended_lane_cur_car_behind_opt)) {
      future_car_to_follow = predictions.at(*intended_lane_cur_car_behind_opt);
    }
  }

  // When we prepare for the lane change, we want our speed to be approximately equal to the speed of the car in the 
  // intended lane. However, if our final lane is not our intended lane, this is when, in fact, our final lane equals to
  // our current lane, we risk making an accident with the car which is currently ahead of us in our lane if its speed
  // or position is less than that of the car in the intended lane. So, we need to adjust accordingly. 
  if (final_lane_other_car_ahead_opt) {
    const auto& future_final_lane_other_car_ahead = predictions.at(*final_lane_other_car_ahead_opt);
    if (!future_car_to_follow || (future_car_to_follow->S() > future_final_lane_other_car_ahead.S())) {
      future_car_to_follow = future_final_lane_other_car_ahead;
    }
  }
  
  // Finally, when there is no car behind in the intended lane that violates the safety buffer and when the car
  // ahead in the final lane goes faster than the car ahead in the intended lane, we plan to follow the car in the 
  // intended lane.
  if (intended_lane_cur_car_ahead_opt) {
    const auto& future_intended_lane_other_car_ahead = predictions.at(*intended_lane_cur_car_ahead_opt);
    if (!future_car_to_follow || (future_car_to_follow->S() > future_intended_lane_other_car_ahead.S())) {
      future_car_to_follow = future_intended_lane_other_car_ahead;
    }
  }
  
  // If we have a car to follow, plan our car to be at a safe distance behind the car to follow and with a slightly
  // lower speed.
  if (future_car_to_follow) {
    return Car::Builder(ego_car)
             .SetTime(ego_car.T() + t)
             .SetCoordinateS(future_car_to_follow->S() - pp_config_.front_car_buffer_m)
             .SetCoordinateD((ego_car.FinalLane() + 0.5) * pp_config_.lane_width_m)
             .SetVelocityS(future_car_to_follow->Vs() - 0.5)    // set slightly lower speed than the car's ahead
             .SetVelocityD(0.0)
             .SetAccelerationS(0.0)
             .SetAccelerationD(0.0)
           .Build();
  } else {
    return std::nullopt;
  }
}


BehaviorLayer::~BehaviorLayer() = default;

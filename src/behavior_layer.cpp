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


Car BehaviorLayer::Plan(const Car& ego_car) const
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

  return ChooseBestPlannedEgoCar(planned_ego_cars, ego_car, predictions);
}


Car BehaviorLayer::ChooseBestPlannedEgoCar(std::vector<Car>& planned_ego_cars,
                                           const Car& cur_ego_car,
                                           const std::map<Car, Car>& predictions) const
{
  // sort in decreasing order of S coordinate, i.e. a car with the largest S goes first
  std::stable_sort(planned_ego_cars.begin(), planned_ego_cars.end(),
                   [](auto& car1, auto& car2) { return car1.S() > car2.S(); });

  std::vector<std::pair<double, Car>> cost_ego_car_map;

  double min_cost = std::numeric_limits<double>::max();
  auto to_ret = planned_ego_cars[0];
  for (int i = 0; i < planned_ego_cars.size(); ++i) {

    double cost = 0.1 * LaneCost(planned_ego_cars[i], predictions) + // penalize for being in certain lane
                  0.5 * CarAheadCost(planned_ego_cars[i], predictions) +
                  LaneMaxSpeedCost(planned_ego_cars[i], predictions);
    if (min_cost > cost) {
      min_cost = cost;
      to_ret = planned_ego_cars[i];
    }

    std::cout << "COST: " << cost << " CAR:\n" << planned_ego_cars[i] << std::endl;
  }

  return to_ret;
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
  double t_diff = pp_config_.frequency_s;

  double s_max_speed_at_horizon = fmin(
      Calc1DVelocity(ego_car.Vs(), ego_car.As(), pp_config_.max_jerk_mps3, t),
      pp_config_.max_speed_mps
  );
  double s_max_speed_at_horizon_prev = fmin(
      Calc1DVelocity(ego_car.Vs(), ego_car.As(), pp_config_.max_jerk_mps3, t - t_diff),
      pp_config_.max_speed_mps
  );

  double s = Calc1DPosition(static_cast<double>(ego_car.S()), s_max_speed_at_horizon, 0.0, t);
  double s_prev = Calc1DPosition(static_cast<double>(ego_car.S()), s_max_speed_at_horizon_prev, 0.0, t - t_diff);

  double d = (ego_car.FinalLane() + 0.5) * pp_config_.lane_width_m; // + 0.5 means lane center

  double vs = Calc1DVelocity(s_prev, s, t_diff);
  double vd = 0.0;

  double time = ego_car.T() + t;

  return Car::Builder(ego_car)
           .SetTime(time)
           .SetCoordinateS(s)
           .SetCoordinateD(d)
           .SetVelocityS(vs)
           .SetVelocityD(vd)
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
             .SetVelocityS(future_car_to_follow->Vs() - 0.5)
             .SetVelocityD(0.0)
             .SetAccelerationS(0.0)
             .SetAccelerationD(0.0)
           .Build();
  } else {
    return std::nullopt;
  }
}


BehaviorLayer::~BehaviorLayer() = default;

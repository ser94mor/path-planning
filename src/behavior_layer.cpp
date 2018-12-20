//
// Created by aoool on 28.10.18.
//

#include "behavior_layer.hpp"
#include "cost_functions.hpp"

#include <typeinfo>
#include <cassert>


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
    planned_ego_cars.push_back(PlanForState(cur_ego_car, predictions));
  }

  assert(!planned_ego_cars.empty());

  return ChooseBestPlannedEgoCar(planned_ego_cars, map_vals(predictions));
}


Car BehaviorLayer::ChooseBestPlannedEgoCar(const std::vector<Car>& ego_cars, const std::vector<Car>& other_cars) const
{
  std::vector<std::pair<double, Car>> cost_ego_car_map;

  double min_cost = std::numeric_limits<double>::max();
  Car to_ret;
  for (const auto& car : ego_cars) {
    double cost = //CarAheadCost(car, other_cars) +
                  LaneMaxSpeedCost(car, other_cars) +
                  3 * BufferViolationCost(car, other_cars);
    if (min_cost > cost) {
      min_cost = cost;
      to_ret = car;
    }
  }

  return to_ret;
}


Car BehaviorLayer::PlanForState(const Car& ego_car, const std::map<Car, Car>& predictions) const
{
  // first, assume that there are no obstacles and plan in accordance with this assumption
  Car planned_ego_car_no_obstacles = PlanWithNoObstacles(ego_car, pp_config_.behavior_planning_time_horizon_s);
  std::optional<Car> planned_ego_car_with_obstacles =
      PlanWithObstacles(ego_car, predictions, pp_config_.behavior_planning_time_horizon_s);

  if (planned_ego_car_with_obstacles.has_value() &&
      planned_ego_car_with_obstacles.value().S() <= planned_ego_car_no_obstacles.S()) {
    return planned_ego_car_with_obstacles.value();
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
BehaviorLayer::PlanWithObstacles(const Car& ego_car, const std::map<Car, Car>& predictions, double t) const
{
  // find another car that is directly ahead of the ego vehicle
  auto&& all_cur_cars = map_keys(predictions);
  std::optional<Car> intended_lane_cur_car_ahead_opt = ego_car.NearestCarAheadInIntendedLane(all_cur_cars);
  std::optional<Car> cur_other_car_ahead_final_lane_opt = ego_car.NearestCarAheadInFinalLane(all_cur_cars);

  std::optional<Car> planned_ego_car{std::nullopt};

  if (intended_lane_cur_car_ahead_opt.has_value()) {
    auto& future_other_car_intended_lane_ahead = predictions.at(intended_lane_cur_car_ahead_opt.value());

    double vel_s = future_other_car_intended_lane_ahead.Vs() - 0.5;
    double vel_d = 0.0;
    planned_ego_car = std::make_optional(
        Car::Builder(ego_car)
          .SetTime(ego_car.T() + t)
          .SetCoordinateS(future_other_car_intended_lane_ahead.S() - pp_config_.front_car_buffer_m)
          .SetCoordinateD((ego_car.FinalLane() + 0.5) * pp_config_.lane_width_m)
          .SetVelocityS(vel_s)
          .SetVelocityD(vel_d)
          .SetAccelerationS(0.0)
          .SetAccelerationD(0.0)
        .Build()
    );
  }

  if (cur_other_car_ahead_final_lane_opt.has_value()) {
    auto& future_other_car_final_lane_ahead = predictions.at(cur_other_car_ahead_final_lane_opt.value());

    if (!planned_ego_car.has_value() ||
        (planned_ego_car.has_value() &&
        (planned_ego_car.value().IsFrontBufferViolatedBy(future_other_car_final_lane_ahead) ||
        planned_ego_car.value().IsInFrontOf(future_other_car_final_lane_ahead)))) {
      double vel_s = future_other_car_final_lane_ahead.Vs() - 0.5;
      double vel_d = 0.0;
      planned_ego_car = std::make_optional(
          Car::Builder(ego_car)
            .SetTime(ego_car.T() + t)
            .SetCoordinateS(future_other_car_final_lane_ahead.S() - pp_config_.front_car_buffer_m)
            .SetCoordinateD((ego_car.FinalLane() + 0.5) * pp_config_.lane_width_m)
            .SetVelocityS(vel_s)
            .SetVelocityD(vel_d)
            .SetAccelerationS(0.0)
            .SetAccelerationD(0.0)
          .Build()
      );
    }
  }


  return planned_ego_car;
}


BehaviorLayer::~BehaviorLayer() = default;

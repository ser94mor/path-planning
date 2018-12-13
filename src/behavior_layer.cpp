//
// Created by aoool on 28.10.18.
//

#include "behavior_layer.hpp"

#include <typeinfo>

BehaviorLayer::BehaviorLayer(const PathPlannerConfig& path_planner_config,
                             LocalizationLayer& localization_layer,
                             PredictionLayer& prediction_layer):
  pp_config_{path_planner_config},
  localization_layer_{localization_layer},
  prediction_layer_{prediction_layer}
{

}



Car BehaviorLayer::Plan(const Car& ego_car)
{
  auto predictions =
      prediction_layer_.GetPredictions(pp_config_.behavior_planning_time_horizon_s, ego_car.time_s);

  const auto& possible_next_states = FSM::GetPossibleNextStates(ego_car.state);

  std::vector<Car> planned_cars{possible_next_states.size()};
  for (int i = 0; i < possible_next_states.size(); ++i) {
    planned_cars[i] = PlanForState(possible_next_states[i], ego_car, predictions);
  }

  return planned_cars[0];


}

Car
BehaviorLayer::PlanForState(const State state, const Car& ego_car,
                            const std::map<Car, Car>& predictions)
{
  switch (state)
  {
    case State::KeepLane:
      return PlanForKeepLaneState(ego_car, predictions);

    case State::PrepareLaneChangeLeft:
      return PlanForPrepareLaneChangeLeftState(ego_car, predictions);

    case State::PrepareLaneChangeRight:
      return PlanForPrepareLaneChangeRightState(ego_car, predictions);

    case State::LaneChangeLeft:
      return PlanForLaneChangeLeftState(ego_car, predictions);

    case State::LaneChangeRight:
      return PlanForLaneChangeRightState(ego_car, predictions);

    default:
      std::cerr  << "BehaviorLayer::PlanForState() for " << typeid(state).name()
                 <<  " must handle " << state << " case" << std::endl;
      std::exit(EXIT_FAILURE);
  }
}


Car BehaviorLayer::PlanForKeepLaneStateAndNoObstacles(const Car& ego_car) {
  double t = pp_config_.behavior_planning_time_horizon_s;
  double t_diff = pp_config_.frequency_s;

  double s_max_speed_at_horizon = fmin(
      Calc1DVelocity(ego_car.vel_s_mps, ego_car.acc_s_mps2, pp_config_.max_jerk_mps3, t),
      pp_config_.max_speed_mps
  );
  double s_max_speed_at_horizon_prev = fmin(
      Calc1DVelocity(ego_car.vel_s_mps, ego_car.acc_s_mps2, pp_config_.max_jerk_mps3, t - t_diff),
      pp_config_.max_speed_mps
  );

  double s = Calc1DPosition(static_cast<double>(ego_car.s_m), s_max_speed_at_horizon, 0.0, t);
  double s_prev = Calc1DPosition(static_cast<double>(ego_car.s_m), s_max_speed_at_horizon_prev, 0.0, t - t_diff);

  double corrected_d = ego_car.Lane() * pp_config_.lane_width_m * 1.5;
  double d = Calc1DPosition(corrected_d, 0.0, 0.0, t);
  double d_prev = Calc1DPosition(corrected_d, 0.0, 0.0, t - t_diff);

  double vs = Calc1DVelocity(s_prev, s, t_diff);
  double vd = Calc1DVelocity(d_prev, d, t_diff);

  double velocity = CalcAbsVelocity(vs, vd);

  double time = ego_car.time_s + t;

  return {
           .id = ego_car.id,
           .state = ego_car.state,
           .vel_mps = velocity,
           .time_s = time,
           .s_m = s,
           .d_m = d,
           .vel_s_mps = vs,
           .vel_d_mps = vd,
           .acc_s_mps2 = 0.0,
           .acc_d_mps2 = 0.0,
         };
}


Car
BehaviorLayer::PlanForKeepLaneState(const Car& ego_car, const std::map<Car, Car>& predictions)
{
  // first, assume that there are no obstacles and plan in accordance with this assumption
  Car planned_ego_car = PlanForKeepLaneStateAndNoObstacles(ego_car);

  // find another car that is directly ahead of the ego vehicle
  auto&& cur_other_cars = map_keys(predictions);
  auto&& cur_other_cars_same_lane = GetCarsInLane(ego_car.Lane(), pp_config_.lane_width_m, cur_other_cars);
  std::optional<Car> cur_other_car_ahead =
      GetNearestCarAheadBySCoordIfPresent(ego_car, cur_other_cars_same_lane);

  // check whether the safety front buffer is violated for the planned car without obstacles
  if (cur_other_car_ahead.has_value()) {
    auto& future_other_car_ahead = predictions.at(*cur_other_car_ahead);

    if (planned_ego_car.IsFrontBufferViolatedBy(future_other_car_ahead) || 
        planned_ego_car.IsInFrontOf(future_other_car_ahead)) {
      planned_ego_car.s_m = future_other_car_ahead.s_m - pp_config_.front_car_buffer_m;
      planned_ego_car.vel_s_mps = future_other_car_ahead.vel_s_mps - 0.5;
      planned_ego_car.vel_mps = CalcAbsVelocity(planned_ego_car.vel_s_mps, planned_ego_car.vel_d_mps);
    }
  }

  return planned_ego_car;
}

Car BehaviorLayer::PlanForPrepareLaneChangeLeftState(const Car& ego_car,
                                                           const std::map<Car, Car>& predictions)
{

  // TODO: implement
  return PlanForKeepLaneState(ego_car, predictions);
}

Car BehaviorLayer::PlanForPrepareLaneChangeRightState(const Car& current_car,
                                                            const std::map<Car, Car>& predictions)
{
  // TODO: implement
  return PlanForKeepLaneState(current_car, predictions);
}

Car BehaviorLayer::PlanForLaneChangeLeftState(const Car& current_car,
                                                    const std::map<Car, Car>& predictions)
{
  return Car();
}

Car BehaviorLayer::PlanForLaneChangeRightState(const Car& current_car,
                                                     const std::map<Car, Car>& predictions)
{
  return Car();
}


BehaviorLayer::~BehaviorLayer() = default;

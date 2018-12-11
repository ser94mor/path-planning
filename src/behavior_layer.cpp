//
// Created by aoool on 28.10.18.
//

#include "behavior_layer.hpp"

BehaviorLayer::BehaviorLayer(const PathPlannerConfig& path_planner_config,
                             LocalizationLayer& localization_layer,
                             PredictionLayer& prediction_layer):
  pp_config_{path_planner_config},
  localization_layer_{localization_layer},
  prediction_layer_{prediction_layer}
{

}



FrenetCar BehaviorLayer::Plan(const FrenetCar& ego_car)
{
  auto predictions =
      prediction_layer_.GetPredictions(pp_config_.behavior_planning_time_horizon_s, ego_car.time_s);

  const auto& possible_next_states = FSM::GetPossibleNextStates(ego_car.state);

  std::vector<FrenetCar> planned_cars{possible_next_states.size()};
  for (int i = 0; i < possible_next_states.size(); ++i) {
    planned_cars[i] = PlanForState(possible_next_states[i], ego_car, predictions);
  }

  return planned_cars[0];


}

FrenetCar
BehaviorLayer::PlanForState(const State state, const FrenetCar& ego_car,
                            const std::map<FrenetCar, FrenetCar>& predictions)
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


FrenetCar BehaviorLayer::PlanForKeepLaneStateAndNoObstacles(const FrenetCar& ego_car) {
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

  double d = Calc1DPosition(ego_car.d_m, 0.0, 0.0, t);
  double d_prev = Calc1DPosition(ego_car.d_m, 0.0, 0.0, t - t_diff);

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


FrenetCar
BehaviorLayer::PlanForKeepLaneState(const FrenetCar& ego_car, const std::map<FrenetCar, FrenetCar>& predictions)
{
  // first, assume that there are no obstacles and plan in accordance with this assumption
  FrenetCar planned_ego_car = PlanForKeepLaneStateAndNoObstacles(ego_car);

  // find another car that is directly ahead of the ego vehicle
  int ego_car_lane = CalcLaneNumber(ego_car.d_m, pp_config_.lane_width_m);
  auto&& cur_other_cars = map_keys(predictions);
  auto&& cur_other_cars_same_lane = GetFrenetCarsInLane(ego_car_lane, pp_config_.lane_width_m, cur_other_cars);
  std::optional<FrenetCar> cur_other_car_ahead =
      GetNearestFrenetCarAheadBySCoordIfPresent(ego_car, cur_other_cars_same_lane);

  // check whether the safety front buffer is violated for the planned car without obstacles
  if (cur_other_car_ahead) {
    auto& future_other_car_ahead = predictions.at(*cur_other_car_ahead);

    if (planned_ego_car.LongitudinalForwardDistanceTo(future_other_car_ahead) < pp_config_.front_car_buffer_m) {
      planned_ego_car.s_m = future_other_car_ahead.s_m - pp_config_.front_car_buffer_m;
      planned_ego_car.vel_s_mps = future_other_car_ahead.vel_s_mps;
      planned_ego_car.vel_mps = future_other_car_ahead.vel_mps;

      std::cout << "CAR AHEAD Detected" << std::endl;
    }
  }

  return planned_ego_car;
}

FrenetCar BehaviorLayer::PlanForPrepareLaneChangeLeftState(const FrenetCar& ego_car,
                                                           const std::map<FrenetCar, FrenetCar>& predictions)
{

  // TODO: implement
  return PlanForKeepLaneState(ego_car, predictions);
}

FrenetCar BehaviorLayer::PlanForPrepareLaneChangeRightState(const FrenetCar& current_car,
                                                            const std::map<FrenetCar, FrenetCar>& predictions)
{
  // TODO: implement
  return PlanForKeepLaneState(current_car, predictions);
}

FrenetCar BehaviorLayer::PlanForLaneChangeLeftState(const FrenetCar& current_car,
                                                    const std::map<FrenetCar, FrenetCar>& predictions)
{
  return FrenetCar();
}

FrenetCar BehaviorLayer::PlanForLaneChangeRightState(const FrenetCar& current_car,
                                                     const std::map<FrenetCar, FrenetCar>& predictions)
{
  return FrenetCar();
}


BehaviorLayer::~BehaviorLayer() = default;

//
// Created by aoool on 28.10.18.
//

#include "behavior_layer.hpp"

BehaviorLayer::BehaviorLayer(const PathPlannerConfig& path_planner_config,
                             LocalizationLayer& localization_layer,
                             PredictionLayer& prediction_layer):
  path_planner_config_{path_planner_config},
  localization_layer_{localization_layer},
  prediction_layer_{prediction_layer}
{

}

FrenetCar BehaviorLayer::Plan(const FrenetCar& current_car)
{
  auto predictions = prediction_layer_.GetPredictions(path_planner_config_.behavior_planning_time_horizon_s);

  const auto& possible_next_states = FSM::GetPossibleNextStates(current_car.state);

  double t = path_planner_config_.behavior_planning_time_horizon_s;
  double t_diff = path_planner_config_.frequency_s;

  double s = Calc1DPosition(current_car.s_m, current_car.vel_s_mps, current_car.acc_s_mps2, t);
  double s_prev = Calc1DPosition(current_car.s_m, current_car.vel_s_mps, current_car.acc_s_mps2, t - t_diff);

  double d = Calc1DPosition(current_car.d_m, current_car.vel_d_mps, current_car.acc_d_mps2, t);
  double d_prev = Calc1DPosition(current_car.d_m, current_car.vel_d_mps, current_car.acc_d_mps2, t - t_diff);

  double vs = Calc1DSpeed(s_prev, s, t_diff);
  double vd = Calc1DSpeed(d_prev, d, t_diff);

  double velocity = CalcAbsVelocity(vs, vd);

  return {
      .id = current_car.id,
      .state = current_car.state,
      .vel_mps = velocity,
      .s_m = s,
      .d_m = d,
      .vel_s_mps = vs,
      .vel_d_mps = vd,
      .acc_s_mps2 = current_car.acc_s_mps2,
      .acc_d_mps2 = current_car.acc_d_mps2,
  };
}

BehaviorLayer::~BehaviorLayer() = default;

//
// Created by aoool on 28.10.18.
//

#include "behavior_layer.hpp"

BehaviorLayer::BehaviorLayer(const PathPlannerConfig& path_planner_config,
                             PredictionLayer& prediction_layer,
                             LocalizationLayer& localization_layer):
  path_planner_config_{path_planner_config},
  prediction_layer_{prediction_layer},
  localization_layer_{localization_layer}
{

}

Car BehaviorLayer::Plan(const Car& current_car) {
  auto predictions = prediction_layer_.GetPredictions(path_planner_config_.behavior_planning_time_horizon_s);

  const auto& possible_next_states = FSM::GetPossibleNextStates(current_car.state);

  double t = path_planner_config_.behavior_planning_time_horizon_s;
  double vel = path_planner_config_.max_speed_mps;

  double s = Calc1DPosition(current_car.s_m, vel, 0.0, t);
  double s_prev = Calc1DPosition(current_car.s_m, vel, 0.0, t - path_planner_config_.frequency_s);
  double d = current_car.d_m;

  auto pos_2d = GetXY(s, d, path_planner_config_);

  auto pos_2d_prev = GetXY(s_prev, d, path_planner_config_);

  double x = pos_2d[0];
  double y = pos_2d[1];

  double vel_x = Calc1DSpeed(pos_2d_prev[0], x, path_planner_config_.frequency_s);
  double vel_y = Calc1DSpeed(pos_2d_prev[1], y, path_planner_config_.frequency_s);

  double yaw = CalcYawRad(vel_x, vel_y);

  auto frenet_speed = GetFrenetSpeed(s, d, x, y, vel_x, vel_y, path_planner_config_);
  double vel_s = frenet_speed[0];
  double vel_d = frenet_speed[1];

  return {
      .id = current_car.id,
      .state = current_car.state,
      .vel_mps = vel,
      .yaw_rad = yaw,
      .x_m = x,
      .y_m = y,
      .vel_x_mps = vel_x,
      .vel_y_mps = vel_y,
      .acc_x_mps2 = 0.0,
      .acc_y_mps2 = 0.0,
      .s_m = s,
      .d_m = d,
      .vel_s_mps = vel_s,
      .vel_d_mps = vel_d,
      .acc_s_mps2 = 0.0,
      .acc_d_mps2 = 0.0,
  };
}

BehaviorLayer::~BehaviorLayer() = default;

//
// Created by aoool on 28.10.18.
//

#include "prediction_layer.hpp"
#include "helpers.hpp"

#include <vector>

PredictionLayer::PredictionLayer(const PathPlannerConfig& path_planner_config, LocalizationLayer& localization_layer):
    path_planner_config_{path_planner_config},
    localization_layer_{localization_layer},
    predictions_{},
    last_update_info_{0, 0.0},
    update_cnt_{0}
{}

std::pair<Car, Car> PredictionLayer::GetPredictionForCar(const Car& car, double t) const {
  double s = Calc1DPosition(car.s_m, car.vel_mps, 0.0, t);
  double s_prev = Calc1DPosition(car.s_m, car.vel_mps, 0.0, t - path_planner_config_.frequency_s);
  double d = car.d_m;

  auto pos_2d = GetXY(s, d, path_planner_config_);

  auto pos_2d_prev = GetXY(s_prev, d, path_planner_config_);

  double x = pos_2d[0];
  double y = pos_2d[1];

  double vel_x = Calc1DSpeed(pos_2d_prev[0], x, path_planner_config_.frequency_s);
  double vel_y = Calc1DSpeed(pos_2d_prev[1], y, path_planner_config_.frequency_s);
  double vel   = Calc2DVectorLen(vel_x, vel_y);

  double yaw = CalcYawRad(vel_x, vel_y);

  auto frenet_speed = GetFrenetSpeed(s, d, x, y, vel_x, vel_y, path_planner_config_);
  double vel_s = frenet_speed[0];
  double vel_d = frenet_speed[1];

  return
    {
      car,
      {
        .id        = car.id,
        .state     = car.state,
        .vel_mps   = vel,
        .yaw_rad   = yaw,
        .x_m       = x,
        .y_m       = y,
        .vel_x_mps = vel_x,
        .vel_y_mps = vel_y,
        .acc_x_mps2 = 0.0,
        .acc_y_mps2 = 0.0,
        .s_m       = s,
        .d_m       = d,
        .vel_s_mps = vel_s,
        .vel_d_mps = vel_d,
        .acc_s_mps2 = 0.0,
        .acc_d_mps2 = 0.0,
      },
    };
}

std::map<Car, Car> PredictionLayer::GetPredictionsForCars(const std::vector<Car>& cars, double t) const {
  std::map<Car, Car> pred_cars{};

  for (int i = 0; i < pred_cars.size(); ++i) {
    pred_cars.insert(GetPredictionForCar(cars[i], t));
  }

  return pred_cars;
}

std::map<Car, Car> PredictionLayer::GetPredictions(double t) {
  return GetUpdateCntPredictionsPair(t).second;
}

std::pair<uint64_t, std::map<Car, Car> > PredictionLayer::GetUpdateCntPredictionsPair(double t) {
  const auto cars = localization_layer_.GetUpdateCntCarsPair();

  if ( !(last_update_info_.first == cars.first && last_update_info_.second == t) ) {
    predictions_.clear();
    predictions_ = GetPredictionsForCars(cars.second, t);
    last_update_info_ = { cars.first, t };
    ++update_cnt_;
  }

  return { update_cnt_, predictions_ };
}

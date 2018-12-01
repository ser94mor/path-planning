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

std::pair<FrenetCar, FrenetCar>
PredictionLayer::GetPredictionForFrenetCar(const FrenetCar& car, double t) const
{
  const double time_diff = path_planner_config_.frequency_s;

  double s = Calc1DPosition(car.s_m, car.vel_s_mps, car.acc_s_mps2, t);
  double s_prev = Calc1DPosition(car.s_m, car.vel_s_mps, car.acc_s_mps2, t - time_diff);

  double d = Calc1DPosition(car.d_m, car.vel_d_mps, car.acc_d_mps2, t);
  double d_prev = Calc1DPosition(car.d_m, car.vel_d_mps, car.acc_d_mps2, t - time_diff);

  double vs = Calc1DSpeed(s_prev, s, time_diff);
  double vd = Calc1DSpeed(d_prev, d, time_diff);
  double velocity = CalcAbsVelocity(vs, vd);

  return
    {
      car,
      {
        .id        = car.id,
        .state     = car.state,
        .vel_mps   = velocity,
        .s_m       = s,
        .d_m       = d,
        .vel_s_mps = vs,
        .vel_d_mps = vd,
        .acc_s_mps2 = car.acc_s_mps2,
        .acc_d_mps2 = car.acc_d_mps2,
      },
    };
}

std::map<FrenetCar, FrenetCar>
PredictionLayer::GetPredictionsForFrenetCars(const std::vector<FrenetCar>& cars, double t) const {
  std::map<FrenetCar, FrenetCar> pred_cars{};

  for (int i = 0; i < pred_cars.size(); ++i) {
    pred_cars.insert(GetPredictionForFrenetCar(cars[i], t));
  }

  return pred_cars;
}

std::map<FrenetCar, FrenetCar> PredictionLayer::GetPredictions(double t) {
  return GetUpdateCntPredictionsPair(t).second;
}

std::pair<uint64_t, std::map<FrenetCar, FrenetCar> > PredictionLayer::GetUpdateCntPredictionsPair(double t) {
  const auto cars = localization_layer_.GetUpdateCntFrenetCarsPair();

  if ( !(last_update_info_.first == cars.first && last_update_info_.second == t) ) {
    predictions_.clear();
    predictions_ = GetPredictionsForFrenetCars(cars.second, t);
    last_update_info_ = { cars.first, t };
    ++update_cnt_;
  }

  return { update_cnt_, predictions_ };
}

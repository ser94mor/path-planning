//
// Created by aoool on 28.10.18.
//

#include "prediction_layer.hpp"
#include "helpers.hpp"

#include <vector>

PredictionLayer::PredictionLayer(const PathPlannerConfig& path_planner_config, LocalizationLayer& localization_layer):
    path_planner_config_{path_planner_config},
    localization_layer_{localization_layer},
    cache_predictions_{5},
    last_update_info_{0, 0.0, 0.0, 0.0}
{

}

std::pair<Car, Car>
PredictionLayer::GetPredictionForCar(const Car& car, double t) const
{
  const double time_diff = path_planner_config_.frequency_s;

  double s = Calc1DPosition(static_cast<double>(car.S()), car.Vs(), car.As(), t);
  double s_prev = Calc1DPosition(static_cast<double>(car.S()), car.Vs(), car.As(), t - time_diff);

  double d = Calc1DPosition(car.D(), car.Vd(), car.Ad(), t);
  double d_prev = Calc1DPosition(car.D(), car.Vd(), car.Ad(), t - time_diff);

  double vs = Calc1DVelocity(s_prev, s, time_diff);
  double vd = Calc1DVelocity(d_prev, d, time_diff);
  double velocity = CalcAbsVelocity(vs, vd);

  double time = car.T() + t;

  return { car,
           CarBuilder()
             .SetId(car.Id())
             .SetState(car.State())
             .SetTime(time)
             .SetCoordinateS(s)
             .SetCoordinateD(d)
             .SetVelocityS(vs)
             .SetVelocityD(vd)
             .SetAccelerationS(car.As())
             .SetAccelerationD(car.Ad())
           .Build(),
         };
}

std::map<Car, Car>
PredictionLayer::GetPredictionsForCars(const std::vector<Car>& cars, double t) const {
  std::map<Car, Car> pred_cars{};

  for (const auto& car : cars) {
    pred_cars.insert(GetPredictionForCar(car, t));
  }

  return pred_cars;
}

std::map<Car, Car> PredictionLayer::GetPredictions(double pred_time, double start_time)
{
  auto cars = localization_layer_.GetCars();
  if (cars.empty()) {
    return {};
  }

  last_update_info_ = { cars.size(), cars[0].T(), pred_time, start_time, };

  if (cache_predictions_.exists(last_update_info_)) {
    return cache_predictions_.get(last_update_info_);
  }

  double offset_time = start_time - cars[0].T();
  if (!IsEqual(offset_time, 0.0)) {
    auto offset_car_preds = GetPredictionsForCars(cars, offset_time);
    for (auto& car : cars) {
      car = offset_car_preds.at(car);
    }
  }

  auto predictions = GetPredictionsForCars(cars, pred_time);
  cache_predictions_.put(last_update_info_, predictions);

  return predictions;
}

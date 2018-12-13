//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_PREDICTION_LAYER_HPP
#define PATH_PLANNING_PREDICTION_LAYER_HPP

#include "localization_layer.hpp"
#include "path_planner_config.hpp"

#include <map>
#include <lrucache.hpp>

class PredictionLayer {

public:
  PredictionLayer(const PathPlannerConfig& path_planner_config, LocalizationLayer& localization_layer);

  std::pair<Car, Car> GetPredictionForCar(const Car& car, double t) const;

  std::map<Car, Car> GetPredictionsForCars(const std::vector<Car>& cars, double t) const;

  std::map<Car, Car> GetPredictions(double pred_time, double start_time);

private:
  const PathPlannerConfig& path_planner_config_;
  LocalizationLayer& localization_layer_;

  cache::lru_cache< std::tuple<size_t, double, double, double>, std::map<Car, Car> > cache_predictions_;

  std::tuple<size_t, double, double, double> last_update_info_;

};

#endif //PATH_PLANNING_PREDICTION_LAYER_HPP

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

  std::pair<FrenetCar, FrenetCar> GetPredictionForFrenetCar(const FrenetCar& car, double t) const;

  std::map<FrenetCar, FrenetCar> GetPredictionsForFrenetCars(const std::vector<FrenetCar>& cars, double t) const;

  std::map<FrenetCar, FrenetCar> GetPredictions(double pred_time, double start_time);

private:
  const PathPlannerConfig& path_planner_config_;
  LocalizationLayer& localization_layer_;

  cache::lru_cache< std::tuple<size_t, double, double, double>, std::map<FrenetCar, FrenetCar> > cache_predictions_;

  std::tuple<size_t, double, double, double> last_update_info_;

};

#endif //PATH_PLANNING_PREDICTION_LAYER_HPP

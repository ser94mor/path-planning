//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_PREDICTION_LAYER_HPP
#define PATH_PLANNING_PREDICTION_LAYER_HPP

#include "localization_layer.hpp"
#include "path_planner_config.hpp"

#include <map>

class PredictionLayer {

public:
  PredictionLayer(const PathPlannerConfig& path_planner_config, LocalizationLayer& localization_layer);

  std::pair<FrenetCar, FrenetCar> GetPredictionForFrenetCar(const FrenetCar& car, double t) const;

  std::map<FrenetCar, FrenetCar> GetPredictionsForFrenetCars(const std::vector<FrenetCar>& cars, double t) const;

  std::map<FrenetCar, FrenetCar> GetPredictions(double t);

  std::pair< uint64_t, std::map<FrenetCar, FrenetCar> > GetUpdateCntPredictionsPair(double t);

private:
  const PathPlannerConfig& path_planner_config_;
  LocalizationLayer& localization_layer_;

  std::map<FrenetCar, FrenetCar> predictions_;
  std::pair<uint64_t, double> last_update_info_;
  uint64_t update_cnt_;

};


#endif //PATH_PLANNING_PREDICTION_LAYER_HPP

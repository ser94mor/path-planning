//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_BEHAVIOR_LAYER_HPP
#define PATH_PLANNING_BEHAVIOR_LAYER_HPP

#include "prediction_layer.hpp"
#include "localization_layer.hpp"
#include "path_planner_config.hpp"


class BehaviorLayer {

public:

  BehaviorLayer(const PathPlannerConfig& path_planner_config,
                PredictionLayer& prediction_layer,
                LocalizationLayer& localization_layer);

  virtual ~BehaviorLayer();

  Car Plan(const Car& current_car);

private:
  const PathPlannerConfig& path_planner_config_;
  PredictionLayer&         prediction_layer_;
  LocalizationLayer&       localization_layer_;
};


#endif //PATH_PLANNING_BEHAVIOR_LAYER_HPP

//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_BEHAVIOR_LAYER_HPP
#define PATH_PLANNING_BEHAVIOR_LAYER_HPP

#include "prediction_layer.hpp"
#include "localization_layer.hpp"


class BehaviorLayer {

public:

  BehaviorLayer(PredictionLayer* prediction_layer, LocalizationLayer* localization_layer);

  virtual ~BehaviorLayer();

private:
  PredictionLayer* prediction_layer_;
  LocalizationLayer* localization_layer_;
};


#endif //PATH_PLANNING_BEHAVIOR_LAYER_HPP

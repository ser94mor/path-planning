//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_PREDICTION_LAYER_HPP
#define PATH_PLANNING_PREDICTION_LAYER_HPP

#include "localization_layer.hpp"

class PredictionLayer {

public:
  PredictionLayer(const LocalizationLayer& localization_layer);

  

private:
  LocalizationLayer localization_layer_;

};


#endif //PATH_PLANNING_PREDICTION_LAYER_HPP

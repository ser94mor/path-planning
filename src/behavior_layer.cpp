//
// Created by aoool on 28.10.18.
//

#include "behavior_layer.hpp"

BehaviorLayer::BehaviorLayer(PredictionLayer *prediction_layer, LocalizationLayer *localization_layer):
  prediction_layer_{prediction_layer},
  localization_layer_{localization_layer}
{

}

BehaviorLayer::~BehaviorLayer() = default;

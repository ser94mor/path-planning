//
// Created by aoool on 28.10.18.
//

#include "localization_layer.hpp"

LocalizationLayer::LocalizationLayer(): sensor_fusion_{0}, cars_{sensor_fusion_.size()}, cars_updated_{false} {}

LocalizationLayer::~LocalizationLayer() = default;

void LocalizationLayer::Update(const std::vector< std::vector<double> >& sensor_fusion) {
  sensor_fusion_ = sensor_fusion;
  cars_updated_ = false;
}

const std::vector<Car>& LocalizationLayer::GetCars() {
  if (not cars_updated_) {
    cars_.resize(sensor_fusion_.size());
    for (int i = 0; i < sensor_fusion_.size(); ++i) {
      cars_[i] = Car::FromVector(sensor_fusion_[i]);
    }
  }

  return cars_;
}

//
// Created by aoool on 28.10.18.
//

#include "localization_layer.hpp"

LocalizationLayer::LocalizationLayer(const PathPlannerConfig& config):
    path_planner_config_{config}, sensor_fusion_{0}, cars_{sensor_fusion_.size()}, cars_updated_{false}, update_cnt_{0}
{

}

LocalizationLayer::~LocalizationLayer() = default;

void LocalizationLayer::Update(const std::vector< std::vector<double> >& sensor_fusion) {
  sensor_fusion_ = sensor_fusion;
  cars_updated_ = false;
}

std::vector<FrenetCar> LocalizationLayer::GetFrenetCars() {
  return GetUpdateCntFrenetCarsPair().second;
}

std::pair< uint64_t, std::vector<FrenetCar> > LocalizationLayer::GetUpdateCntFrenetCarsPair() {
  if (not cars_updated_) {
    cars_.resize(sensor_fusion_.size());
    for (int i = 0; i < sensor_fusion_.size(); ++i) {
      cars_[i] = FrenetCar::FromVectorAssumingConstantVelocityAndLaneKeeping(sensor_fusion_[i], path_planner_config_);
    }
    ++update_cnt_;
  }

  return { update_cnt_, cars_ };
}

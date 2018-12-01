//
// Created by aoool on 15.11.18.
//

#ifndef PATH_PLANNING_TRAJECTORY_LAYER_HPP
#define PATH_PLANNING_TRAJECTORY_LAYER_HPP

#include "path_planner_config.hpp"
#include "localization_layer.hpp"
#include "prediction_layer.hpp"
#include "behavior_layer.hpp"

#include <vector>

class TrajectoryLayer {
public:
  TrajectoryLayer(const PathPlannerConfig& config,
                  LocalizationLayer& localization_layer,
                  PredictionLayer& prediction_layer,
                  BehaviorLayer& behavior_layer);

  virtual ~TrajectoryLayer();

  std::vector< std::vector<double> > GetTrajectory(const FrenetCar& car) const;

private:
  std::vector<double> GetJerkMinimizingTrajectory(std::vector<double> start, std::vector<double> end, double t);

  const PathPlannerConfig& path_planner_config_;
  LocalizationLayer& localization_layer_;
  PredictionLayer& prediction_layer_;
  BehaviorLayer& behavior_layer_;
};


#endif //PATH_PLANNING_TRAJECTORY_LAYER_HPP

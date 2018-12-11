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
#include <deque>

class TrajectoryLayer {
public:
  TrajectoryLayer(const PathPlannerConfig& config,
                  LocalizationLayer& localization_layer,
                  PredictionLayer& prediction_layer,
                  BehaviorLayer& behavior_layer);

  virtual ~TrajectoryLayer();

  void Initialize(const FrenetCar& car);


  std::vector<FrenetCar> GetTrajectory(size_t num_points);

private:
  std::vector<double> GetJerkMinimizingTrajectory(std::vector<double> start, std::vector<double> end, double t) const;

  const PathPlannerConfig& pp_config_;
  LocalizationLayer& localization_layer_;
  PredictionLayer& prediction_layer_;
  BehaviorLayer& behavior_layer_;

  bool initialized_;
  FrenetCar ego_car_;
  std::deque<FrenetCar> next_cars_;
};


#endif //PATH_PLANNING_TRAJECTORY_LAYER_HPP

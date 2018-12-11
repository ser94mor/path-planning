//
// Created by aoool on 8/8/18.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "car.hpp"
#include "speed_controller.hpp"
#include "path_planner_config.hpp"
#include "trajectory_layer.hpp"
#include "localization_layer.hpp"
#include "prediction_layer.hpp"
#include "behavior_layer.hpp"

#include <vector>

class PathPlanner {

public:

  PathPlanner(PathPlannerConfig config, PIDControllerConfig pid_config);

  virtual ~PathPlanner();

  /**
   * Generate X and Y tracks for the car, that is, plan its path.
   * @return vector containing 2 vectors with X and Y coordinates correspondingly
   */
  std::vector< std::vector< double > >& GetNextXYTrajectories(const FrenetCar& current_ego_car,
                                                              const std::vector<double>& prev_path_x_m,
                                                              const std::vector<double>& prev_path_y_m,
                                                              const std::vector< std::vector<double> >& sensor_fusion);
  
  const PathPlannerConfig& GetPathPlannerConfig() const;

private:

  PathPlannerConfig config_;
  std::vector< std::vector<double> > next_coords_;
  bool invoked_;
  FrenetCar car_;
  SpeedController speed_ctrl_;

  LocalizationLayer localization_layer_;
  PredictionLayer prediction_layer_;
  BehaviorLayer behavior_layer_;
  TrajectoryLayer trajectory_layer_;

};

#endif //PATH_PLANNING_PATHPLANNER_H

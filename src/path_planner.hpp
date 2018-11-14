//
// Created by aoool on 8/8/18.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "car.hpp"
#include "speed_controller.hpp"
#include "path_planner_config.hpp"

#include <vector>

class PathPlanner {

public:

  PathPlanner(PathPlannerConfig config, PIDControllerConfig pid_config);

  virtual ~PathPlanner();

  /**
   * Generate X and Y tracks for the car, that is, plan its path.
   * @return vector containing 2 vectors with X and Y coordinates correspondingly
   */
  std::vector< std::vector< double > >& GetNextXYTrajectories(Car& car,
                                                              std::vector<double>& prev_path_x_m,
                                                              std::vector<double>& prev_path_y_m,
                                                              std::vector< std::vector<double> >& sensor_fusion);

private:

  std::vector<double> GetPrevXY(Car &car,
                                std::vector<double> &prev_path_x,
                                std::vector<double> &prev_path_y,
                                int back_offset);

  PathPlannerConfig config_;
  std::vector< std::vector<double> > next_coords_;
  bool invoked_;
  double target_speed_mps_;
  double prev_acc_mps2_;
  double prev_s_m_;
  double prev_d_m_;
  double prev_speed_mps_;
  SpeedController speed_ctrl_;

};

#endif //PATH_PLANNING_PATHPLANNER_H

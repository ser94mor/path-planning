//
// Created by aoool on 8/8/18.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "Car.hpp"
#include "CarState.hpp"
#include "PID.hpp"
#include "PathPlannerConfig.hpp"

#include <cstdlib>
#include <vector>


class PathPlanner {

public:

  PathPlanner(PathPlannerConfig config);

  /**
   * Generate X and Y tracks for the car, that is, plan its path.
   * @return vector containing 2 vectors with X and Y coordinates correspondingly
   */
  std::vector< std::vector< double > > &GetNextXYTrajectories(Car &car,
                                                              std::vector<double> &prev_path_x_m,
                                                              std::vector<double> &prev_path_y_m,
                                                              std::vector< std::vector<double> > &sensor_fusion);

  virtual ~PathPlanner();

private:

  std::vector<double> GetPrevXY(Car &car,
                                std::vector<double> &prev_path_x,
                                std::vector<double> &prev_path_y,
                                int back_offset);

  CarState GetCarState(Car &car,
                       std::vector<double> &prev_path_x,
                       std::vector<double> &prev_path_y);

  PathPlannerConfig config_;
  std::vector< std::vector<double> > next_coords_;
  bool invoked_;
  double target_speed_mps_;
  double prev_acc_mps2_;
  double prev_s_m_;
  double prev_d_m_;
  double prev_speed_mps_;
  PID speed_pid_ctrl_;

};

#endif //PATH_PLANNING_PATHPLANNER_H

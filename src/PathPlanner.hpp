//
// Created by aoool on 8/8/18.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
#include "Car.hpp"
#include "CarState.hpp"

class PathPlanner {

public:

  PathPlanner(double frequency_s,
              double target_speed_mps,
              double max_acceleration_mps2,
              double max_jerk_mps3,
              size_t path_len,
              std::vector<double> const &map_waypoints_x_m,
              std::vector<double> const &map_waypoints_y_m,
              std::vector<double> const &map_waypoints_s_m,
              std::vector<double> const &map_waypoints_d_x_m,
              std::vector<double> const &map_waypoints_d_y_m);

  /**
   * Generate X and Y tracks for the car, that is, plan its path.
   * @return vector containing 2 vectors with X and Y coordinates correspondingly
   */
  std::vector< std::vector< double > > &GetNextXYTrajectories(Car &car,
                                                              std::vector<double> &prev_path_x_m,
                                                              std::vector<double> &prev_path_y_m,
                                                              double end_path_s_m,
                                                              double end_path_d_m,
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

  double freq_s_;
  double target_speed_mps_;
  double max_acc_mps2_;
  double max_jerk_mps3_;
  size_t path_len_;
  std::vector< std::vector<double> > next_coords_;
  std::vector<double> const &map_waypoints_x_m_;
  std::vector<double> const &map_waypoints_y_m_;
  std::vector<double> const &map_waypoints_s_m_;
  std::vector<double> const &map_waypoints_d_x_m_;
  std::vector<double> const &map_waypoints_d_y_m_;
  bool invoked_;

};

#endif //PATH_PLANNING_PATHPLANNER_H

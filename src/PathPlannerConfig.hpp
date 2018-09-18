//
// Created by aoool on 9/3/18.
//

#ifndef PATH_PLANNING_PATHPLANNERCONFIG_H
#define PATH_PLANNING_PATHPLANNERCONFIG_H

#include <vector>
#include <cstdlib>

struct PathPlannerConfig {
  double frequency_s;
  double max_speed_mps;
  double max_acc_mps2;
  double max_jerk_mps3;
  size_t path_len;
  size_t num_lanes;
  double lane_width_m;
  std::vector<double> map_waypoints_x_m;
  std::vector<double> map_waypoints_y_m;
  std::vector<double> map_waypoints_s_m;
  std::vector<double> map_waypoints_d_x_m;
  std::vector<double> map_waypoints_d_y_m;
};

#endif //PATH_PLANNING_PATHPLANNERCONFIG_H

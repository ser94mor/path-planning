//
// Created by aoool on 12.11.18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_CONFIG_HPP
#define PATH_PLANNING_PATH_PLANNER_CONFIG_HPP

#include <vector>
#include <cstdlib>

struct PathPlannerConfig {
  double frequency_s;
  double min_speed_mps;
  double max_speed_mps;
  double min_acc_mps2;
  double max_acc_mps2;
  double min_jerk_mps3;
  double max_jerk_mps3;
  size_t path_len;
  size_t num_lanes;
  double lane_width_m;
  double max_s_m;

  std::vector<double> map_waypoints_x_m;
  std::vector<double> map_waypoints_y_m;
  std::vector<double> map_waypoints_s_m;
  std::vector<double> map_waypoints_dx_m;
  std::vector<double> map_waypoints_dy_m;

  static PathPlannerConfig FromFile(const char *pp_file, const char *map_file);
};

#endif //PATH_PLANNING_PATH_PLANNER_CONFIG_HPP

//
// Created by aoool on 12.11.18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_CONFIG_HPP
#define PATH_PLANNING_PATH_PLANNER_CONFIG_HPP

#include <vector>
#include <cstdlib>
#include <spline.h>


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
  double behavior_planning_time_horizon_s;

  std::vector<double> map_wps_x_m;
  std::vector<double> map_wps_y_m;
  std::vector<double> map_wps_s_m;
  std::vector<double> map_wps_dx_m;
  std::vector<double> map_wps_dy_m;

  tk::spline spline_s_x;
  tk::spline spline_s_y;
  tk::spline spline_s_dx;
  tk::spline spline_s_dy;

  void InitSplines();

  static PathPlannerConfig FromFile(const char *pp_file, const char *map_file);

private:

  static std::vector<double> PrepareSPointsForSpline(double max_s, const std::vector<double>& map_wps_s);
  static std::vector<double> PrepareCartesianCoordinateForSpline(const std::vector<double>& map_wps_coord);
};

#endif //PATH_PLANNING_PATH_PLANNER_CONFIG_HPP

//
// Created by aoool on 12.11.18.
//

#ifndef PATH_PLANNING_PATH_PLANNER_CONFIG_HPP
#define PATH_PLANNING_PATH_PLANNER_CONFIG_HPP

#include <vector>
#include <cstdlib>
#include <iostream>
#include <spline.hpp>


struct PathPlannerConfig {
  double frequency_s;
  double min_speed_mps;
  double max_speed_mps;
  double min_acc_mps2;
  double max_acc_mps2;
  double min_jerk_mps3;
  double max_jerk_mps3;
  size_t path_len;
  size_t trajectory_layer_queue_len;
  size_t num_lanes;
  double lane_width_m;
  double max_s_m;
  double behavior_planning_time_horizon_s;
  double front_car_buffer_m;
  double back_car_buffer_m;
  double side_car_buffer_m;
  double region_of_interest_front_m;
  double region_of_interest_back_m;

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


inline std::ostream& operator<<(std::ostream& ostream, const PathPlannerConfig& config)
{
  ostream << std::fixed
          << "PathPlannerConfig{\n"
             "  .frequency_s                      = " << config.frequency_s                      << ",\n"
             "  .min_speed_mps                    = " << config.min_speed_mps                    << ",\n"
             "  .max_speed_mps                    = " << config.max_speed_mps                    << ",\n"
             "  .min_acc_mps2                     = " << config.min_acc_mps2                     << ",\n"
             "  .max_acc_mps2                     = " << config.max_acc_mps2                     << ",\n"
             "  .min_jerk_mps3                    = " << config.min_jerk_mps3                    << ",\n"
             "  .max_jerk_mps3                    = " << config.max_jerk_mps3                    << ",\n"
             "  .path_len                         = " << config.path_len                         << ",\n"
             "  .trajectory_layer_queue_len       = " << config.trajectory_layer_queue_len       << ",\n"
             "  .num_lanes                        = " << config.num_lanes                        << ",\n"
             "  .lane_width_m                     = " << config.lane_width_m                     << ",\n"
             "  .max_s_m                          = " << config.max_s_m                          << ",\n"
             "  .behavior_planning_time_horizon_s = " << config.behavior_planning_time_horizon_s << ",\n"
             "  .front_car_buffer_m               = " << config.front_car_buffer_m               << ",\n"
             "  .back_car_buffer_m                = " << config.back_car_buffer_m                << ",\n"
             "  .side_car_buffer_m                = " << config.side_car_buffer_m                << ",\n"
             "  .region_of_interest_front_m       = " << config.region_of_interest_front_m       << ",\n"
             "  .region_of_interest_back_m        = " << config.region_of_interest_back_m        << ",\n"
             "  .map_wps_*_m.size()               = " << config.map_wps_x_m.size()               << ",\n"
          << "}";

  return ostream;
}

#endif //PATH_PLANNING_PATH_PLANNER_CONFIG_HPP

//
// Created by aoool on 12.11.18.
//

#include "path_planner_config.hpp"
#include "helpers.hpp"

#include <json.hpp>
#include <fstream>

// for convenience
using json = nlohmann::json;

PathPlannerConfig PathPlannerConfig::FromFile(const char* pp_file, const char* map_file) {

  // read path planner configuration
  std::ifstream config_ifstream(pp_file);
  json config_json;
  config_ifstream >> config_json;

  PathPlannerConfig config = {
      .frequency_s = config_json["frequency_s"].get<double>(),
      .min_speed_mps = MphToMps(config_json["min_speed_mph"].get<double>()),
      .max_speed_mps = MphToMps(config_json["max_speed_mph"].get<double>()),
      .min_acc_mps2 = config_json["min_acc_mps2"].get<double>(),
      .max_acc_mps2 = config_json["max_acc_mps2"].get<double>(),
      .min_jerk_mps3 = config_json["min_jerk_mps3"].get<double>(),
      .max_jerk_mps3 = config_json["max_jerk_mps3"].get<double>(),
      .path_len = config_json["path_len"].get<size_t>(),
      .num_lanes = config_json["num_lanes"].get<size_t>(),
      .lane_width_m = config_json["lane_width_m"].get<double>(),
      .max_s_m = config_json["max_s_m"].get<double>(),
      .behavior_planning_time_horizon_s = config_json["behavior_planning_time_horizon_s"].get<double>(),
      .map_wps_x_m = {},
      .map_wps_y_m = {},
      .map_wps_s_m = {},
      .map_wps_dx_m = {},
      .map_wps_dy_m = {},
      .spline_s_x = {},
      .spline_s_y = {},
      .spline_s_dx = {},
      .spline_s_dy = {},
  };

  // read highway map and initialize arrays
  std::ifstream map_csv(map_file, std::ifstream::in);
  std::string line;
  while (getline(map_csv, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    config.map_wps_x_m.push_back(x);
    config.map_wps_y_m.push_back(y);
    config.map_wps_s_m.push_back(s);
    config.map_wps_dx_m.push_back(d_x);
    config.map_wps_dy_m.push_back(d_y);
  }

  config.InitSplines();

  return config;
}

std::vector<double> PathPlannerConfig::PrepareSPointsForSpline(double max_s, const std::vector<double>& map_wps_s)
{
  std::vector<double> s_pts(map_wps_s.size() + 2);
  std::copy(map_wps_s.begin(), map_wps_s.end(), s_pts.begin() + 1);
  s_pts[0] = map_wps_s[map_wps_s.size() - 1] - max_s;
  s_pts[s_pts.size() - 1] = max_s;

  return s_pts;
}

std::vector<double> PathPlannerConfig::PrepareCartesianCoordinateForSpline(const std::vector<double>& map_wps_coord)
{
  std::vector<double> coord_pts(map_wps_coord.size() + 2);
  std::copy(map_wps_coord.begin(), map_wps_coord.end(), coord_pts.begin() + 1);
  coord_pts[0] = map_wps_coord[map_wps_coord.size() - 1];
  coord_pts[coord_pts.size() - 1] = map_wps_coord[0];

  return coord_pts;
}

void PathPlannerConfig::InitSplines() {
  auto s_pts  = PrepareSPointsForSpline(this->max_s_m, this->map_wps_s_m);
  auto x_pts  = PrepareCartesianCoordinateForSpline(this->map_wps_x_m);
  auto y_pts  = PrepareCartesianCoordinateForSpline(this->map_wps_y_m);
  auto dx_pts = PrepareCartesianCoordinateForSpline(this->map_wps_dx_m);
  auto dy_pts = PrepareCartesianCoordinateForSpline(this->map_wps_dy_m);

  this->spline_s_x.set_points(s_pts, x_pts);
  this->spline_s_y.set_points(s_pts, y_pts);
  this->spline_s_dx.set_points(s_pts, dx_pts);
  this->spline_s_dy.set_points(s_pts, dy_pts);
}

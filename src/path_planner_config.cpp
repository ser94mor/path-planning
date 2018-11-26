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
      .behavior_planning_time_horizon = config_json["behavior_planning_time_horizon"].get<double>(),
      .map_wps_x_m = {},
      .map_wps_y_m = {},
      .map_wps_s_m = {},
      .map_wps_dx_m = {},
      .map_wps_dy_m = {},
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

  return config;
}

//
// Created by aoool on 14.11.18.
//

#include "pid_controller_config.hpp"

#include <cstdlib>
#include <fstream>
#include <json.hpp>

// for convenience
using json = nlohmann::json;

PIDControllerConfig PIDControllerConfig::FromFile(const char *pid_file) {
  // read PID controller configuration
  std::ifstream config_ifstream(pid_file);
  json config_json;
  config_ifstream >> config_json;

  PIDControllerConfig config = {
      .Kp_initial = config_json["Kp_initial"].get<double>(),
      .Ki_initial = config_json["Ki_initial"].get<double>(),
      .Kd_initial = config_json["Kd_initial"].get<double>(),
      .twiddle_dKp_initial = config_json["twiddle_dKp_initial"].get<double>(),
      .twiddle_dKi_initial = config_json["twiddle_dKi_initial"].get<double>(),
      .twiddle_dKd_initial = config_json["twiddle_dKd_initial"].get<double>(),
      .delay_before_calc_tot_error = config_json["delay_before_calc_tot_error"].get<int>(),
      .frequency_of_coeff_tuning = config_json["frequency_of_coeff_tuning"].get<int>(),
      .stop_threshold = config_json["stop_threshold"].get<double>(),
  };

  return config;
}

//
// Created by aoool on 14.11.18.
//

#ifndef PATH_PLANNING_PID_CONTROLLER_CONFIG_HPP
#define PATH_PLANNING_PID_CONTROLLER_CONFIG_HPP

#include <iostream>

struct PIDControllerConfig {
  double Kp_initial;
  double Ki_initial;
  double Kd_initial;
  double twiddle_dKp_initial;
  double twiddle_dKi_initial;
  double twiddle_dKd_initial;
  int    delay_before_calc_tot_error; // measured in iterations
  int    frequency_of_coeff_tuning;   // measured in iterations
  double stop_threshold;              // stop TWIDDLE when the sum of coefficients is less than or equal to this value

  static PIDControllerConfig FromFile(const char *pid_file);
};


inline std::ostream& operator<<(std::ostream& ostream, const PIDControllerConfig& config)
{
  ostream << std::fixed
          << "PIDControllerConfig{\n"
             "  .Kp_initial                  = " << config.Kp_initial                  << ",\n"
             "  .Ki_initial                  = " << config.Ki_initial                  << ",\n"
             "  .Kd_initial                  = " << config.Kd_initial                  << ",\n"
             "  .twiddle_dKp_initial         = " << config.twiddle_dKp_initial         << ",\n"
             "  .twiddle_dKi_initial         = " << config.twiddle_dKi_initial         << ",\n"
             "  .twiddle_dKd_initial         = " << config.twiddle_dKd_initial         << ",\n"
             "  .delay_before_calc_tot_error = " << config.delay_before_calc_tot_error << ",\n"
             "  .frequency_of_coeff_tuning   = " << config.frequency_of_coeff_tuning   << ",\n"
             "  .stop_threshold              = " << config.stop_threshold              << ",\n"
          << "}";

  return ostream;
}

#endif //PATH_PLANNING_PID_CONTROLLER_CONFIG_HPP

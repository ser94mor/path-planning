//
// Created by aoool on 14.11.18.
//

#ifndef PATH_PLANNING_PID_CONTROLLER_CONFIG_HPP
#define PATH_PLANNING_PID_CONTROLLER_CONFIG_HPP


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



#endif //PATH_PLANNING_PID_CONTROLLER_CONFIG_HPP

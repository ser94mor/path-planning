#ifndef PATH_PLANNING_PIDCONTROLLER_HPP
#define PATH_PLANNING_PIDCONTROLLER_HPP

#include "pid_controller_config.hpp"

#include <cstdint>


class PIDController {
public:

  explicit PIDController(PIDControllerConfig config);

  virtual ~PIDController();

  // Updates the PID error variables given cross track error.
  void UpdateError(double error);

  // Calculates the control signal.
  double ControlSignal() const;

private:

  // Configuration defining the controller internal behavior.
  PIDControllerConfig config_;

  // Errors
  double p_error_;
  double i_error_;
  double d_error_;

  // Coefficients
  double Kp_;
  double Ki_;
  double Kd_;

  // Helper variables for TWIDDLE algorithm
  double dKp_;
  double dKi_;
  double dKd_;
  double best_err_;
  double total_err_;
  int    coeff_ind_;
  int    attempts_per_ind_;

  // Tunes parameters in accordance with the "TWIDDLE" algorithm for parameter tuning.
  void TuneCoeffsUsingTwiddleAlg();

  double& IndToCoeff(int ind);
  double& IndToDeltaCoeff(int ind);

};

#endif /* PATH_PLANNING_PIDCONTROLLER_HPP */

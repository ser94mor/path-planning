#include "pid_controller.hpp"

#include <iostream>
#include <limits>

PIDController::PIDController(PIDControllerConfig config):
    config_{config},
    p_error_{0.0},
    i_error_{0.0},
    d_error_{0.0},
    Kp_{config_.Kp_initial},
    Ki_{config_.Ki_initial},
    Kd_{config_.Kd_initial},
    dKp_{config_.twiddle_dKp_initial},
    dKi_{config_.twiddle_dKi_initial},
    dKd_{config_.twiddle_dKd_initial},
    best_err_{std::numeric_limits<double>::max()},
    total_err_{0.0},
    coeff_ind_{0},
    attempts_per_ind_{0}
{

}

PIDController::~PIDController() = default;

void PIDController::UpdateError(double error) {
  static uint64_t step_cnt = 1; // iterations counter

  // update differential cross track error
  d_error_ = error - p_error_;  // here p_error_ plays role of the previous cross track error
  // update proportional cross track error
  p_error_ = error;
  // update integral cross track error
  i_error_ += error;

  // increment step counter, since we have processed the observation
  ++step_cnt;

  // calculate total error; PID controller needs some time to "stabilize," so it makes sense not to calculate
  // total error while we only started
  if (step_cnt >= config_.delay_before_calc_tot_error) {
    total_err_ += error * error;
  }

  if (config_.frequency_of_coeff_tuning && (step_cnt % config_.frequency_of_coeff_tuning) == 0) {
    TuneCoeffsUsingTwiddleAlg();
  }
}

double &PIDController::IndToCoeff(int ind) {
  ind %= 3;
  switch(ind) {
    case 0:
      return Kp_;
    case 1:
      return Ki_;
    case 2:
      return Kd_;
    default:
      std::cerr << "Unexpected coefficient index in TWIDDLE algorithm" << std:: endl;
      std::exit(1);
  }
}

double &PIDController::IndToDeltaCoeff(int ind) {
  switch(ind) {
    case 0:
      return dKp_;
    case 1:
      return dKi_;
    case 2:
      return dKd_;
    default:
      std::cerr << "Unexpected delta coefficient index in TWIDDLE algorithm" << std:: endl;
      std::exit(2);
  }
}

void PIDController::TuneCoeffsUsingTwiddleAlg() {
  if (dKp_ + dKi_ + dKd_ > config_.stop_threshold) {
    switch (attempts_per_ind_) {

      case 0:
        IndToCoeff(coeff_ind_) += IndToDeltaCoeff(coeff_ind_);
        break;

      case 1:
        if (total_err_ < best_err_) {
          best_err_ = total_err_;
          IndToDeltaCoeff(coeff_ind_) *= 1.1;
          attempts_per_ind_ = 2; // this value will trigger the switch to the next coeff. (see code below)
        } else {
          IndToCoeff(coeff_ind_) -= 2.0 * IndToDeltaCoeff(coeff_ind_);
        }
        break;

      case 2:
        if (total_err_ < best_err_) {
          best_err_ = total_err_;
          IndToDeltaCoeff(coeff_ind_) *= 1.1;
          attempts_per_ind_ = 2; // this value will trigger the switch to the next coeff. (see code below)
        } else {
          IndToCoeff(coeff_ind_) += IndToDeltaCoeff(coeff_ind_);
          IndToDeltaCoeff(coeff_ind_) *= 0.9;
        }
        break;

      default:
        std::cerr << "attempts_per_ind_ must not exceed the value of 2" << std::endl;
        std::exit(3);
    }

    total_err_ = 0.0;
    p_error_ = 0.0;
    i_error_ = 0.0;
    d_error_ = 0.0;

    if (coeff_ind_ == 2) {
      coeff_ind_ = 0;
      attempts_per_ind_ = 0;
    } else if (attempts_per_ind_ == 2) {
      ++coeff_ind_;
      attempts_per_ind_ = 0;
    } else {
      ++attempts_per_ind_;
    }

  }
}

double PIDController::ControlSignal() const {
  return Kp_ * p_error_ + Ki_ * i_error_ + Kd_ * d_error_;
}

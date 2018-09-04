#include "PID.h"

#include <iostream>
#include <limits>


using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, unsigned int calc_tot_err_after, unsigned int tune_coeffs_each,
               double twiddle_dKp_initial, double twiddle_dKi_initial, double twiddle_dKd_initial,
               double twiddle_stops_when) {
  // coeffs. and errors
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->p_error = 0.0;
  this->i_error = 0.0;
  this->d_error = 0.0;

  // helper variables
  this->_total_err = 0.0;
  this->_step_cnt = 0;
  this->_tune_coeffs_each = tune_coeffs_each;
  this->_calc_tot_err_after = calc_tot_err_after;

  // twiddle helpers
  this->_dKp = (Kp == 0.0 ? 0.0 : twiddle_dKp_initial); // the initial zero value of the coeff. signifies
  this->_dKi = (Ki == 0.0 ? 0.0 : twiddle_dKi_initial); // that we do not want to use P/I/D control
  this->_dKd = (Kd == 0.0 ? 0.0 : twiddle_dKd_initial); //
  this->_stop_when = twiddle_stops_when;
  this->_best_err = std::numeric_limits<double>::max();
  this->_coeff_ind = 0;
  this->_attempts_per_ind = 0;
}

void PID::UpdateError(double cte) {
  // update differential cross track error
  d_error = cte - p_error;  // here p_error plays role of the previous cross track error
  // update proportional cross track error
  p_error = cte;
  // update integral cross track error
  i_error += cte;

  // increment step counter, since we have processed the observation
  ++_step_cnt;

  // calculate total error; PID controller needs some time to "stabilize," so it makes sense not to calculate
  // total error while we only started
  if (_step_cnt >= _calc_tot_err_after) {
    _total_err += cte * cte;
  }

  if (_tune_coeffs_each && (_step_cnt % _tune_coeffs_each) == 0) {
    TuneCoeffsUsingTwiddleAlg();
  }
}

double &PID::IndToCoeff(int ind) {
  ind %= 3;
  switch(ind) {
    case 0:
      return Kp;
    case 1:
      return Ki;
    case 2:
      return Kd;
    default:
      std::cerr << "Unexpected coefficient index in TWIDDLE algorithm" << std:: endl;
      exit(1);
  }
}

double &PID::IndToDeltaCoeff(int ind) {
  switch(ind) {
    case 0:
      return _dKp;
    case 1:
      return _dKi;
    case 2:
      return _dKd;
    default:
      std::cerr << "Unexpected delta coefficient index in TWIDDLE algorithm" << std:: endl;
      exit(2);
  }
}

void PID::TuneCoeffsUsingTwiddleAlg() {
  if (_dKp + _dKi + _dKd > _stop_when) {
    switch (_attempts_per_ind) {

      case 0:
        IndToCoeff(_coeff_ind) += IndToDeltaCoeff(_coeff_ind);
        break;

      case 1:
        if (_total_err < _best_err) {
          _best_err = _total_err;
          IndToDeltaCoeff(_coeff_ind) *= 1.1;
          _attempts_per_ind = 2; // this value will trigger the switch to the next coeff. (see code below)
        } else {
          IndToCoeff(_coeff_ind) -= 2.0 * IndToDeltaCoeff(_coeff_ind);
        }
        break;

      case 2:
        if (_total_err < _best_err) {
          _best_err = _total_err;
          IndToDeltaCoeff(_coeff_ind) *= 1.1;
          _attempts_per_ind = 2; // this value will trigger the switch to the next coeff. (see code below)
        } else {
          IndToCoeff(_coeff_ind) += IndToDeltaCoeff(_coeff_ind);
          IndToDeltaCoeff(_coeff_ind) *= 0.9;
        }
        break;

      default:
        std::cerr << "_attempts_per_ind must not exceed the value of 2" << std::endl;
        exit(3);
    }

    _total_err = 0.0;
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;

    if (_coeff_ind == 2) {
      _coeff_ind = 0;
      _attempts_per_ind = 0;
    } else if (_attempts_per_ind == 2) {
      ++_coeff_ind;
      _attempts_per_ind = 0;
    } else {
      ++_attempts_per_ind;
    }

  }
}

double PID::Correction() const {
  double correction = -Kp * p_error - Ki * i_error - Kd * d_error;

  // values of the characteristic under control belongs to [-1.0, 1.0]
  if (correction > 1.0) {
    correction = 1.0;
  } else if (correction < -1.0) {
    correction = -1.0;
  }

  return correction;
}


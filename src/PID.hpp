#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, unsigned int calc_tot_err_after, unsigned int tune_coeffs_each,
            double twiddle_dKp_initial, double twiddle_dKi_initial, double twiddle_dKd_initial,
            double twiddle_stops_when);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the correction value.
  */
  double Correction() const;

private:

  /*
  * Helper variables
  */
  unsigned int _calc_tot_err_after;
  unsigned int _tune_coeffs_each;
  unsigned long long _step_cnt;

  /*
  * Helper variables for TWIDDLE algorithm
  */
  double _dKp;
  double _dKi;
  double _dKd;
  double _stop_when;
  double _best_err;
  double _total_err;
  int _coeff_ind;
  int _attempts_per_ind;

  /*
  * Tune parameters in accordance with the "TWIDDLE" algorithm for parameter tuning.
  */
  void TuneCoeffsUsingTwiddleAlg();

  double &IndToCoeff(int ind);
  double &IndToDeltaCoeff(int ind);

};

#endif /* PID_H */

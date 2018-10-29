//
// Created by aoool on 29.10.18.
//

#ifndef PATH_PLANNING_SPEEDCONTROLLER_HPP
#define PATH_PLANNING_SPEEDCONTROLLER_HPP

#include "pid_controller.hpp"


class SpeedController {

public:

  SpeedController(double max_vel,  double min_vel,
                  double max_acc,  double min_acc,
                  double max_jerk, double min_jerk,
                  PIDControllerConfig pid_config);

  virtual ~SpeedController();

  double GetVelocity(double target_speed, double current_speed, double time);

private:
  // min. and max. bounds
  double max_vel_;
  double min_vel_;
  double max_acc_;
  double min_acc_;
  double max_jerk_;
  double min_jerk_;

  // remembering the previous state
  double prev_acc_;

  PIDController pid_controller_;
};


#endif //PATH_PLANNING_SPEEDCONTROLLER_HPP

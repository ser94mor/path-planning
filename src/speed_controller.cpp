//
// Created by aoool on 29.10.18.
//

#include "speed_controller.hpp"


SpeedController::~SpeedController() = default;

SpeedController::SpeedController(double max_vel,  double min_vel,  double max_acc, double min_acc,
                                 double max_jerk, double min_jerk, PIDControllerConfig pid_config):
    max_vel_{max_vel},
    min_vel_{min_vel},
    max_acc_{max_acc},
    min_acc_{min_acc},
    max_jerk_{max_jerk},
    min_jerk_{min_jerk},
    prev_acc_{0.0},
    pid_controller_{pid_config}
{

}

double SpeedController::GetVelocity(double target_speed, double current_speed, double time) {

  // use PID controller to get the acceleration
  double error = target_speed - current_speed;
  pid_controller_.UpdateError(error);

  // ensure that we do not violate the acceleration limits
  double acceleration = pid_controller_.ControlSignal();
  if (acceleration > max_acc_) {
    acceleration = max_acc_;
  } else if (acceleration < min_acc_) {
    acceleration = min_acc_;
  }

  // ensure that we do not violate the jerk limits
  double jerk = (acceleration - prev_acc_) / time;
  if (jerk > max_jerk_) {
    acceleration = prev_acc_ + max_jerk_ * time;
  } else if (jerk < min_jerk_) {
    acceleration = prev_acc_ + min_jerk_* time;
  }

  // ensure that we do not violate the velocity limits
  double velocity = current_speed + acceleration * time;
  if (velocity > max_vel_) {
    velocity = max_vel_;
  } else if (velocity < min_vel_) {
    velocity = min_vel_;
  }

  // calculate the acceleration; it may be different than the one calculated above if we adjusted the velocity
  prev_acc_ = (velocity - current_speed) / time;

  return velocity;

}

//
// Created by aoool on 14.11.18.
//

#include "car.hpp"

const PathPlannerConfig* FrenetCar::pp_config_ = nullptr;

Car Car::FromVector(const std::vector<double>& car_info, const PathPlannerConfig& config) {
  double vel = sqrt(car_info[3]*car_info[3] + car_info[4]*car_info[4]);
  double yaw = CalcYawRad(car_info[3], car_info[4]);
  double x = car_info[1];
  double y = car_info[2];
  double vel_x = car_info[3];
  double vel_y = car_info[4];
  double s = car_info[5];
  double d = car_info[6];
  auto frenet_vel = GetFrenetSpeed(s, d, x, y, vel_x, vel_y, config);
  double vel_s = frenet_vel[0];
  double vel_d = frenet_vel[1];

  return {
      .id         = static_cast<int>(car_info[0]),
      .state      = State::KeepLane,
      .vel_mps    = vel,
      .yaw_rad    = yaw,
      .x_m        = x,
      .y_m        = y,
      .vel_x_mps  = vel_x,
      .vel_y_mps  = vel_y,
      .acc_x_mps2 = 0.0,
      .acc_y_mps2 = 0.0,
      .s_m        = s,
      .d_m        = d,
      .vel_s_mps  = vel_s,
      .vel_d_mps  = vel_d,
      .acc_s_mps2 = 0.0,
      .acc_d_mps2 = 0.0,
  };
}

double CartesianCar::GetVelocity() const {
  return sqrt( this->vel_x_mps * this->vel_x_mps + this->vel_y_mps * this->vel_y_mps );
}

double CartesianCar::GetYaw() const {
  return CalcYawRad(this->vel_x_mps, this->vel_y_mps);
}

FrenetCar
FrenetCar::FromVectorAssumingConstantVelocityAndLaneKeeping(const std::vector<double>& car_info,
                                                            const double time,
                                                            const PathPlannerConfig& config)
{
  double velocity = CalcAbsVelocity(car_info[3], car_info[4]);
  return {
      .id = static_cast<int>(car_info[0]),
      .state = State::KeepLane,
      .vel_mps = velocity,
      .time_s = time,
      .s_m = car_info[5],
      .d_m = car_info[6],
      .vel_s_mps = velocity,  // lane keeping
      .vel_d_mps = 0.0,       //
      .acc_s_mps2 = 0.0,      // constant velocity
      .acc_d_mps2 = 0.0,      //
  };
}

double FrenetCar::LongitudinalForwardDistanceTo(const FrenetCar& car) const {
  return static_cast<double>(car.s_m - this->s_m);
}

double FrenetCar::LongitudinalBackwardDistanceTo(const FrenetCar& car) const {
  return static_cast<double>(this->s_m - car.s_m);
}

double FrenetCar::LateralDistanceTo(const FrenetCar& car) const {
  return fabs(this->d_m - car.d_m);
}

void FrenetCar::SetPathPlannerConfig(const PathPlannerConfig* pp_config) {
  pp_config_ = pp_config;
}

bool FrenetCar::IsFrontBufferViolatedBy(const FrenetCar& car) const {
  if (!pp_config_) {
    throw std::logic_error("FrenetCar::pp_config_ pointer must have been initialized by "
                           "FrenetCar::SetPathPlannerConfig before");
  }
  return this->LongitudinalForwardDistanceTo(car) < pp_config_->front_car_buffer_m;
}

bool FrenetCar::IsBackBufferViolatedBy(const FrenetCar& car) const {
  if (!pp_config_) {
    throw std::logic_error("FrenetCar::pp_config_ pointer must have been initialized by "
                           "FrenetCar::SetPathPlannerConfig before");
  }
  return this->LongitudinalBackwardDistanceTo(car) < pp_config_->back_car_buffer_m;
}

bool FrenetCar::IsSideBufferViolatedBy(const FrenetCar& car) const {
  if (!pp_config_) {
    throw std::logic_error("FrenetCar::pp_config_ pointer must have been initialized by "
                           "FrenetCar::SetPathPlannerConfig before");
  }
  return this->LateralDistanceTo(car) < pp_config_->side_car_buffer_m;
}

int FrenetCar::Lane() const {
  return static_cast<int>(floor(this->d_m / pp_config_->lane_width_m));
}

bool FrenetCar::IsInFrontOf(const FrenetCar& car) const {
  return (this->s_m >= car.s_m);
}

bool FrenetCar::IsBehind(const FrenetCar& car) const {
  return (this->s_m < car.s_m);
}

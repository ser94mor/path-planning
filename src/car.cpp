//
// Created by aoool on 14.11.18.
//

#include "car.hpp"

const PathPlannerConfig* Car::pp_config_ = nullptr;


Car Car::FromVectorAssumingConstantVelocityAndLaneKeeping(const std::vector<double>& car_info,
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

double Car::LongitudinalForwardDistanceTo(const Car& car) const {
  return static_cast<double>(car.s_m - this->s_m);
}

double Car::LongitudinalBackwardDistanceTo(const Car& car) const {
  return static_cast<double>(this->s_m - car.s_m);
}

double Car::LateralDistanceTo(const Car& car) const {
  return fabs(this->d_m - car.d_m);
}

void Car::SetPathPlannerConfig(const PathPlannerConfig* pp_config) {
  pp_config_ = pp_config;
}

bool Car::IsFrontBufferViolatedBy(const Car& car) const {
  if (!pp_config_) {
    throw std::logic_error("Car::pp_config_ pointer must have been initialized by "
                           "Car::SetPathPlannerConfig before");
  }
  return this->LongitudinalForwardDistanceTo(car) < pp_config_->front_car_buffer_m;
}

bool Car::IsBackBufferViolatedBy(const Car& car) const {
  if (!pp_config_) {
    throw std::logic_error("Car::pp_config_ pointer must have been initialized by "
                           "Car::SetPathPlannerConfig before");
  }
  return this->LongitudinalBackwardDistanceTo(car) < pp_config_->back_car_buffer_m;
}

bool Car::IsSideBufferViolatedBy(const Car& car) const {
  if (!pp_config_) {
    throw std::logic_error("Car::pp_config_ pointer must have been initialized by "
                           "Car::SetPathPlannerConfig before");
  }
  return this->LateralDistanceTo(car) < pp_config_->side_car_buffer_m;
}

int Car::Lane() const {
  return static_cast<int>(floor(this->d_m / pp_config_->lane_width_m));
}

bool Car::IsInFrontOf(const Car& car) const {
  return (this->s_m >= car.s_m);
}

bool Car::IsBehind(const Car& car) const {
  return (this->s_m < car.s_m);
}

std::vector<Car> Car::CarsInRegionOfInterest(const std::vector<Car>& cars) const {
  std::vector<Car> cars_in_range;
  for (const auto& car: cars) {
    if (car.s_m >= (this->s_m - pp_config_->region_of_interest_back_m) &&
        car.s_m < (this->s_m + pp_config_->region_of_interest_front_m)) {
      cars_in_range.push_back(car);
    }
  }

  return cars_in_range;
}

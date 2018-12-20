#include <utility>

//
// Created by aoool on 8/8/18.
//

#include "car.hpp"
#include "helpers.hpp"
#include "path_planner.hpp"
#include "behavior_layer.hpp"
#include "localization_layer.hpp"
#include "prediction_layer.hpp"

#include <cmath>
#include <cassert>
#include <iostream>
#include <iomanip>
#include <vector>
#include <typeinfo>

std::vector< std::vector<double> >&
PathPlanner::GetNextXYTrajectories(const Car& current_ego_car,
                                   const std::vector<double>& prev_path_x,
                                   const std::vector<double>& prev_path_y,
                                   const std::vector< std::vector<double> >& sensor_fusion)
{
  assert( prev_path_x.size() == prev_path_y.size() );

  static bool sufficiently_fast_moving = false;
  static uint64_t invocation_counter = 0;
  if (invocation_counter == 0) {
    car_ = current_ego_car;
  }

  std::cout << "\n======" << ++invocation_counter << "======\n";

  // update the list of characteristics of other cars
  localization_layer_.Update(sensor_fusion, car_.T());

  auto prev_path_size = prev_path_x.size();

  for (int i = 0; i < prev_path_size; i++) {
    // we do not re-plan the route
    next_coords_[0][i] = prev_path_x[i];
    next_coords_[1][i] = prev_path_y[i];
  }

  std::vector<Car> cars;
  if (sufficiently_fast_moving) {
    cars = trajectory_layer_.GetTrajectory(config_.path_len - prev_path_size);
  }

  for (auto i = 0; i < config_.path_len - prev_path_size; ++i) {

    if (!sufficiently_fast_moving && car_.Vs() >= config_.max_speed_mps) {
      sufficiently_fast_moving = true;
      trajectory_layer_.Initialize(car_);
      cars = trajectory_layer_.GetTrajectory(config_.path_len - prev_path_size);
    }

    if (!sufficiently_fast_moving) {

      // control speed using the PID controller; keep lane
      auto s_prev = car_.S();
      double s_vel_prev = car_.Vs();
      double s_vel = speed_ctrl_.GetVelocity(config_.max_speed_mps, s_vel_prev, config_.frequency_s);


      car_ = Car::Builder(car_)
               .SetTime(current_ego_car.T() + i * config_.frequency_s)
               .SetCoordinateS(s_prev + s_vel * config_.frequency_s)
               .SetVelocityS(s_vel)
               .SetAccelerationS(Calc1DAcc(s_vel_prev, s_vel, config_.frequency_s))
             .Build();

      std::vector<double> coords = GetXY(static_cast<double>(car_.S()), car_.D(), config_);

      next_coords_[0][i + prev_path_size] = coords[0];
      next_coords_[1][i + prev_path_size] = coords[1];

    } else {

      std::vector<double> coords = GetXY(static_cast<double>(cars[i].S()), cars[i].D(), config_);

      next_coords_[0][i + prev_path_size] = coords[0];
      next_coords_[1][i + prev_path_size] = coords[1];

      car_ = cars[i];

    }

  }

  invoked_ = true;


  std::cout << typeid(this).name() << "::GetNextXYTrajectories: last ego car in planned path is\n" << car_ << '\n';
  std::cout << "======" << invocation_counter << "======\n\n" << std::endl;

  return next_coords_;
}


PathPlanner::~PathPlanner() = default;

PathPlanner::PathPlanner(PathPlannerConfig path_config, PIDControllerConfig pid_config):
                         config_{std::move(path_config)},
                         next_coords_{2, std::vector<double>(config_.path_len)},
                         invoked_{false},
                         car_{},
                         speed_ctrl_{config_.max_speed_mps, config_.min_speed_mps,
                                     config_.max_acc_mps2, config_.min_acc_mps2,
                                     config_.max_jerk_mps3, config_.min_jerk_mps3,
                                     pid_config},
                         localization_layer_{config_},
                         prediction_layer_{config_, localization_layer_},
                         behavior_layer_{config_, localization_layer_, prediction_layer_},
                         trajectory_layer_{config_, localization_layer_, prediction_layer_, behavior_layer_}
{
  std::cout << '\n' << config_ << "\n\n" << pid_config << '\n' << std::endl;
}

const PathPlannerConfig& PathPlanner::GetPathPlannerConfig() const {
  return config_;
}

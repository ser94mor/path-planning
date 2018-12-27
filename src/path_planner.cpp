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

  auto prev_path_size = prev_path_x.size();

  static uint64_t invocation_counter = 0;
  if (invocation_counter == 0) {
    car_ = current_ego_car;
    trajectory_layer_.Initialize(car_);
  }

  std::cout << "\n======" << ++invocation_counter << "======\n";

  // update the list of characteristics of other cars
  localization_layer_.Update(sensor_fusion, car_.T());

  for (int i = 0; i < prev_path_size; i++) {
    // we do not re-plan the route
    next_coords_[0][i] = prev_path_x[i];
    next_coords_[1][i] = prev_path_y[i];
  }

  std::vector<Car>  cars = trajectory_layer_.GetTrajectory(config_.path_len - prev_path_size);

  for (auto i = 0; i < config_.path_len - prev_path_size; ++i) {
    std::vector<double> coords = GetXY(static_cast<double>(cars[i].S()), cars[i].D(), config_);

    next_coords_[0][i + prev_path_size] = coords[0];
    next_coords_[1][i + prev_path_size] = coords[1];

    car_ = cars[i];
  }

  std::cout << typeid(this).name() << "::GetNextXYTrajectories: last ego car in planned path is\n" << car_ << '\n';
  std::cout << "======" << invocation_counter << "======\n\n" << std::endl;

  return next_coords_;
}


PathPlanner::~PathPlanner() = default;

PathPlanner::PathPlanner(PathPlannerConfig path_config):
                         config_{std::move(path_config)},
                         next_coords_{2, std::vector<double>(config_.path_len)},
                         car_{},
                         localization_layer_{config_},
                         prediction_layer_{config_, localization_layer_},
                         behavior_layer_{config_, localization_layer_, prediction_layer_},
                         trajectory_layer_{config_, localization_layer_, prediction_layer_, behavior_layer_}
{
  std::cout << '\n' << config_ << '\n' << std::endl;
}

const PathPlannerConfig& PathPlanner::GetPathPlannerConfig() const {
  return config_;
}

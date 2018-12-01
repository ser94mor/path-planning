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

std::vector<double> PathPlanner::GetPrevXY(Car& car,
                                           std::vector<double>& prev_path_x,
                                           std::vector<double>& prev_path_y,
                                           int back_offset) {
  if (not invoked_) {
    // the first invocation of the path planner happens when car does not move, thus all previous coords are the same
    return { car.x_m, car.y_m };
  }

  size_t prev_size = prev_path_x.size();
  if ( prev_size > back_offset ) {

    // coordinates are stored in prev_path_x and prev_path_y vectors
    size_t index = prev_size - back_offset - 1;
    return { prev_path_x[index], prev_path_y[index] };

  } else {

    // coordinates are stored in next_coords_ vector
    size_t index = config_.path_len + prev_size - back_offset - 1;
    return { next_coords_[0][index], next_coords_[1][index] };

  }
}

std::vector< std::vector<double> >&
PathPlanner::GetNextXYTrajectories(const Car& car,
                                   const std::vector<double>& prev_path_x,
                                   const std::vector<double>& prev_path_y,
                                   const std::vector< std::vector<double> >& sensor_fusion)
{
  assert( prev_path_x.size() == prev_path_y.size() );

  static uint64_t invocation_counter = 0;
  if (invocation_counter == 0) {
    prev_car_ = car;
  }

  std::cout << "===" << ++invocation_counter << "===\n";

  // update the list of characteristics of other cars
  localization_layer_.Update(sensor_fusion);

  auto prev_path_size = prev_path_x.size();

  for (int i = 0; i < prev_path_size; i++) {
    // we do not re-plan the route
    next_coords_[0][i] = prev_path_x[i];
    next_coords_[1][i] = prev_path_y[i];
  }

  for (auto i = prev_path_size; i < config_.path_len; i++) {

//    trajectory_layer_.GetJerkMinimizingTrajectory();

    prev_car_.vel_s_mps = speed_ctrl_.GetVelocity(config_.max_speed_mps, prev_car_.vel_s_mps, config_.frequency_s);

    prev_car_.s_m += prev_car_.vel_s_mps * config_.frequency_s;

    if (prev_car_.s_m > config_.max_s_m) {
      prev_car_.s_m -= config_.max_s_m;
    }

    std::vector<double> coords = GetXY(prev_car_.s_m, prev_car_.d_m, config_);

    next_coords_[0][i] = coords[0];
    next_coords_[1][i] = coords[1];
  }

  invoked_ = true;

  return next_coords_;
}


PathPlanner::~PathPlanner() = default;

PathPlanner::PathPlanner(PathPlannerConfig path_config, PIDControllerConfig pid_config):
                         config_{std::move(path_config)},
                         next_coords_{2, std::vector<double>(config_.path_len)},
                         invoked_{false},
                         prev_car_{},
                         speed_ctrl_{config_.max_speed_mps, 0, config_.max_acc_mps2, -config_.max_acc_mps2,
                                     config_.max_jerk_mps3, -config_.max_jerk_mps3, pid_config},
                         localization_layer_{config_},
                         prediction_layer_{config_, localization_layer_},
                         behavior_layer_{config_, localization_layer_, prediction_layer_},
                         trajectory_layer_{config_, localization_layer_, prediction_layer_, behavior_layer_}
{

  std::cout << "|||||||||||||||||||||||||||||||||||||||||||||\n"
            << "||               PATH PLANNER              ||\n"
            << "|| frequency            : " << std::setw(10) << config_.frequency_s   << " s     ||\n"
            << "|| target speed         : " << std::setw(10) << config_.max_speed_mps << " m/s   ||\n"
            << "|| maximum acceleration : " << std::setw(10) << config_.max_acc_mps2  << " m/s^2 ||\n"
            << "|| maximum jerk         : " << std::setw(10) << config_.max_jerk_mps3 << " m/s^3 ||\n"
            << "|| path length          : " << std::setw(10) << config_.path_len      << " -     ||\n"
            << "|| number of lanes      : " << std::setw(10) << config_.num_lanes     << " -     ||\n"
            << "|| lane width           : " << std::setw(10) << config_.lane_width_m  << " m     ||\n"
            << "|||||||||||||||||||||||||||||||||||||||||||||\n"
            << std::endl;

}

const PathPlannerConfig& PathPlanner::GetPathPlannerConfig() const {
  return config_;
}

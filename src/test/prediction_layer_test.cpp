//
// Created by aoool on 02.12.18.
//

#include "prediction_layer.hpp"
#include "car.hpp"

#include <catch.hpp>

static PathPlannerConfig config{
    .frequency_s = 1,
    .min_speed_mps = 2,
    .max_speed_mps = 3,
    .min_acc_mps2 = 4,
    .max_acc_mps2 = 5,
    .min_jerk_mps3 = 6,
    .max_jerk_mps3 = 7,
    .path_len = 8,
    .num_lanes = 9,
    .lane_width_m = 10,
    .max_s_m = 11,
    .behavior_planning_time_horizon_s = 12,
    .map_wps_x_m  = { 1, 2, 3, 4 },
    .map_wps_y_m  = { 1, 2, 3, 4 },
    .map_wps_s_m  = { sqrt(2), sqrt(8), sqrt(18), sqrt(32) },
    .map_wps_dx_m = { sqrt(0.5)+1, sqrt(0.5)+2, sqrt(0.5)+3, sqrt(0.5)+4 },
    .map_wps_dy_m = { -sqrt(0.5)+3, -sqrt(0.5)+6, -sqrt(0.5)+9, -sqrt(0.5)+12 },
    .spline_s_x = {},
    .spline_s_y = {},
    .spline_s_dx = {},
    .spline_s_dy = {},
};

// TODO: write tests for PredictionLayer

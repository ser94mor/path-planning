//
// Created by aoool on 28.10.18.
//

#include "car.hpp"

#include <catch.hpp>
#include <ostream>

TEST_CASE("operator<<(Car)", "[car]") {
  std::ostringstream oss;

  Car car = {
    .id = -1,
    .state = State::PrepareLaneChangeRight,
    .vel_mps = 1.1,
    .yaw_rad = 2.2,
    .x_m = 3.3,
    .y_m = 4.4,
    .vel_x_mps = 5.5,
    .vel_y_mps = 6.6,
    .acc_x_mps2 = 7.7,
    .acc_y_mps2 = 8.8,
    .s_m = 9.9,
    .d_m = 10.10,
    .vel_s_mps = 11.11,
    .vel_d_mps = 12.12,
    .acc_s_mps2 = 13.13,
    .acc_d_mps2 = 14.14,
  };

  std::string expected_res =
      "Car{\n"
      "  .id         = -1,\n"
      "  .state      = PLCR,\n"
      "  .vel_mps    = 1.100000,\n"
      "  .yaw_rad    = 2.200000,\n"
      "  .x_m        = 3.300000,\n"
      "  .y_m        = 4.400000,\n"
      "  .vel_x_mps  = 5.500000,\n"
      "  .vel_y_mps  = 6.600000,\n"
      "  .acc_x_mps2 = 7.700000,\n"
      "  .acc_y_mps2 = 8.800000,\n"
      "  .s_m        = 9.900000,\n"
      "  .d_m        = 10.100000,\n"
      "  .vel_s_mps  = 11.110000,\n"
      "  .vel_d_mps  = 12.120000,\n"
      "  .acc_s_mps2 = 13.130000,\n"
      "  .acc_d_mps2 = 14.140000,\n"
      "}";

  oss << car;

  REQUIRE( oss.str() == expected_res );
}

TEST_CASE("Car::FromVector", "[car]") {
  PathPlannerConfig config{
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
      // in case of Car::FromVector only map waypoints are important
      .map_wps_x_m  = { 0, 1, 2, 3, 4 },
      .map_wps_y_m  = { 0, 1, 2, 3, 4 },
      .map_wps_s_m  = { 0, sqrt(2), sqrt(8), sqrt(18), sqrt(32) },
      .map_wps_dx_m = { sqrt(0.5), sqrt(0.5)+1, sqrt(0.5)+2, sqrt(0.5)+3, sqrt(0.5)+4 },
      .map_wps_dy_m = { -sqrt(0.5), -sqrt(0.5)+3, -sqrt(0.5)+6, -sqrt(0.5)+9, -sqrt(0.5)+12 },
      .spline_s_x = {},
      .spline_s_y = {},
      .spline_s_dx = {},
      .spline_s_dy = {},
  };
  config.InitSplines();

  std::vector<double> car_vect{ 12,            /*id*/
                                2.0,           /*x*/
                                1.0,           /*y*/
                                1.0,           /*vx*/
                                -1.0,          /*vy*/
                                1.5*sqrt(2.0), /*s*/
                                sqrt(2.0)/2,   /*d*/
                              };

  Car car = Car::FromVector(car_vect, config);

  REQUIRE( car.id         == 12 );
  REQUIRE( car.state      == State::KeepLane );
  REQUIRE( car.vel_mps    == Approx(1.4142135624) );
  REQUIRE( car.yaw_rad    == Approx(5.4977871438) );
  REQUIRE( car.x_m        == Approx(2.0) );
  REQUIRE( car.y_m        == Approx(1.0) );
  REQUIRE( car.vel_x_mps  == Approx(1.0) );
  REQUIRE( car.vel_y_mps  == Approx(-1.0) );
  REQUIRE( car.acc_x_mps2 == Approx(0.0) );
  REQUIRE( car.acc_y_mps2 == Approx(0.0) );
  REQUIRE( car.s_m        == Approx(2.1213203436) );
  REQUIRE( car.d_m        == Approx(0.7071067812) );
  REQUIRE( fabs(0.0 - car.vel_s_mps) < 0.0000000001 );
  REQUIRE( car.vel_d_mps  == Approx(1.4142135624) );
  REQUIRE( car.acc_s_mps2 == Approx(0.0) );
  REQUIRE( car.acc_d_mps2 == Approx(0.0) );
}

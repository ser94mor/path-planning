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
    .time_s = 2.2,
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
      "  .time_s     = 2.200000,\n"
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
      .trajectory_layer_queue_len = 9,
      .num_lanes = 10,
      .lane_width_m = 11,
      .max_s_m = 12,
      .behavior_planning_time_horizon_s = 13,
      .front_car_buffer_m = 5.0,
      .back_car_buffer_m = 0.5,
      .side_car_buffer_m = 1.0,
      .region_of_interest_front_m = 50.0,
      .region_of_interest_back_m = 20.0,
      .map_wps_x_m  = {        0.0,          1.0,          2.0,          3.0,           4.0, },
      .map_wps_y_m  = {        0.0,          1.0,          2.0,          3.0,           4.0, },
      .map_wps_s_m  = {        0.0,      sqrt(2),      sqrt(8),     sqrt(18),      sqrt(32), },
      .map_wps_dx_m = {  sqrt(0.5),  sqrt(0.5)+1,  sqrt(0.5)+2,  sqrt(0.5)+3,   sqrt(0.5)+4, },
      .map_wps_dy_m = { -sqrt(0.5), -sqrt(0.5)+3, -sqrt(0.5)+6, -sqrt(0.5)+9, -sqrt(0.5)+12, },
      .spline_s_x = {},
      .spline_s_y = {},
      .spline_s_dx = {},
      .spline_s_dy = {},
  };
  config.InitSplines();

  std::vector<double> car_vect{ 12,              /*id*/
                                1.0,             /*x*/
                                2.0,             /*y*/
                                1.0,             /*vx*/
                                1.0,            /*vy*/
                                1.5*sqrt(2.0),   /*s*/
                                -0.5*sqrt(2.0),   /*d*/
                              };

  Car car = Car::FromVectorAssumingConstantVelocityAndLaneKeeping(car_vect, 2.22, config);

  REQUIRE( car.id         == 12 );
  REQUIRE( car.state      == State::KeepLane );
  REQUIRE( car.vel_mps    == Approx(sqrt(2)) );
  REQUIRE( car.time_s     == Approx(2.22) );
  REQUIRE( car.s_m        == Approx(1.5*sqrt(2.0)) );
  REQUIRE( car.d_m        == Approx(-0.5*sqrt(2.0)) );
  REQUIRE( car.vel_s_mps  == Approx(sqrt(2.0)) );
  REQUIRE( car.vel_d_mps  == Approx(0.0) );
  REQUIRE( car.acc_s_mps2 == Approx(0.0) );
  REQUIRE( car.acc_d_mps2 == Approx(0.0) );
}

TEST_CASE("GetCarsInLane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 4.0, };
  Car::SetPathPlannerConfig(&pp_config);

  std::vector<Car> all_cars(5);
  all_cars[0].d_m = 3;
  all_cars[1].d_m = 6;
  all_cars[2].d_m = 9;
  all_cars[3].d_m = 4.5;
  all_cars[4].d_m = 7.5;

  std::vector<Car> expected_cars_in_lane{ all_cars[1], all_cars[3], all_cars[4] };
  REQUIRE(GetCarsInLane(1, 4, all_cars) == expected_cars_in_lane );

  REQUIRE(GetCarsInLane(3, 4, all_cars).empty() );

  expected_cars_in_lane = { all_cars[0] };
  REQUIRE(GetCarsInLane(0, 4, all_cars) == expected_cars_in_lane );

  expected_cars_in_lane = { all_cars[2] };
  REQUIRE(GetCarsInLane(2, 4, all_cars) == expected_cars_in_lane );
}

TEST_CASE("GetNearestCarAheadBySCoordIfPresent", "car")
{
  Car ego_car{ .s_m = 1.0, };

  auto res = GetNearestCarAheadBySCoordIfPresent(ego_car, {});
  REQUIRE( !res.has_value() );

  Car other_car1{ .s_m = 2.0, };
  res = GetNearestCarAheadBySCoordIfPresent(ego_car, { other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  Car other_car2{ .s_m = 3.0, };
  res = GetNearestCarAheadBySCoordIfPresent(ego_car, { other_car1, other_car2 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  res = GetNearestCarAheadBySCoordIfPresent(ego_car, { other_car2, other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  Car other_car3{ .s_m = 0.5, };
  res = GetNearestCarAheadBySCoordIfPresent(ego_car, { other_car3, other_car2, other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );
}

TEST_CASE("Car::LongitudinalForwardDistanceTo", "[car]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);

  Car ego_car{ .s_m = 0.0, };
  Car other_car{ .s_m = 2.9, };
  REQUIRE( ego_car.LongitudinalForwardDistanceTo(other_car) == Approx(2.9) );

  ego_car.s_m = 2.9;
  other_car.s_m = 0.0;
  REQUIRE( ego_car.LongitudinalForwardDistanceTo(other_car) == Approx(0.1) );

  ego_car.s_m = 2.9;
  other_car.s_m = 0.1;
  REQUIRE( ego_car.LongitudinalForwardDistanceTo(other_car) == Approx(0.2) );

  ego_car.s_m = 0.1;
  other_car.s_m = 2.9;
  REQUIRE( ego_car.LongitudinalForwardDistanceTo(other_car) == Approx(2.8) );
}

TEST_CASE("Car::LongitudinalBackwardDistanceTo", "[car]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);

  Car ego_car{ .s_m = 0.0, };
  Car other_car{ .s_m = 2.9, };
  REQUIRE( ego_car.LongitudinalBackwardDistanceTo(other_car) == Approx(0.1) );

  ego_car.s_m = 2.9;
  other_car.s_m = 0.0;
  REQUIRE( ego_car.LongitudinalBackwardDistanceTo(other_car) == Approx(2.9) );

  ego_car.s_m = 2.9;
  other_car.s_m = 0.1;
  REQUIRE( ego_car.LongitudinalBackwardDistanceTo(other_car) == Approx(2.8) );

  ego_car.s_m = 0.1;
  other_car.s_m = 2.9;
  REQUIRE( ego_car.LongitudinalBackwardDistanceTo(other_car) == Approx(0.2) );
}

TEST_CASE("Car::LateralDistanceTo", "[car]")
{
  Car ego_car{ .d_m = 1.0, };
  Car other_car{ .d_m = 1.0, };

  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.0) );

  other_car.d_m = 0.5;
  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.5) );

  other_car.d_m = 1.5;
  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.5) );
}

TEST_CASE("Car::IsFrontBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config{ .max_s_m = 10.0, };
  Car::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  Car ego_car{ .s_m = 1.0, };
  Car other_car{ .s_m = 2.0, };
  pp_config.front_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car.s_m = 1.0;
  other_car.s_m = 2.0;
  pp_config.front_car_buffer_m = 0.5;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car.s_m = 9.1;
  other_car.s_m = 0.0;
  pp_config.front_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car.s_m = 9.1;
  other_car.s_m = 0.1;
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car.s_m = 9.0;
  other_car.s_m = 0.0;
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car.s_m = 10.0;
  other_car.s_m = 1.0;
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car.s_m = 10.5;
  other_car.s_m = 1.5;
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car.s_m = 10.5;
  other_car.s_m = 1.0;
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( ego_car.IsFrontBufferViolatedBy(other_car) );
}

TEST_CASE("Car::IsBackBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config{ .max_s_m = 10.0, };
  Car::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  Car ego_car{ .s_m = 2.0, };
  Car other_car{ .s_m = 1.0, };
  pp_config.back_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car.s_m = 2.0;
  other_car.s_m = 1.0;
  pp_config.back_car_buffer_m = 0.5;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car.s_m = 0.0;
  other_car.s_m = 9.1;
  pp_config.back_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car.s_m = 0.1;
  other_car.s_m = 9.1;
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car.s_m = 0.0;
  other_car.s_m = 9.0;
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car.s_m = 1.0;
  other_car.s_m = 10.0;
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car.s_m = 1.5;
  other_car.s_m = 10.5;
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car.s_m = 1.0;
  other_car.s_m = 10.5;
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( ego_car.IsBackBufferViolatedBy(other_car) );
}

TEST_CASE("Car::IsSideBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config;
  Car::SetPathPlannerConfig(&pp_config);

  Car ego_car{ .d_m = 1.0, };
  Car other_car{ .d_m = 2.0, };
  pp_config.side_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsSideBufferViolatedBy(other_car) );

  ego_car.d_m = 1.0;
  other_car.d_m = 2.0;
  pp_config.side_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsSideBufferViolatedBy(other_car) );

  ego_car.d_m = 3.0;
  other_car.d_m = 2.0;
  pp_config.side_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsSideBufferViolatedBy(other_car) );

  ego_car.d_m = 3.0;
  other_car.d_m = 2.0;
  pp_config.side_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsSideBufferViolatedBy(other_car) );
}

TEST_CASE("Car::Lane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 3.0 };
  Car::SetPathPlannerConfig(&pp_config);

  REQUIRE( -3 == Car{ .d_m = -8.3, }.Lane() );
  REQUIRE( -2 == Car{ .d_m = -4.0, }.Lane() );
  REQUIRE( -1 == Car{ .d_m = -0.1, }.Lane() );
  REQUIRE(  0 == Car{ .d_m =  2.3, }.Lane() );
  REQUIRE(  1 == Car{ .d_m =  3.1, }.Lane() );
  REQUIRE(  2 == Car{ .d_m =  7.5, }.Lane() );
  REQUIRE(  3 == Car{ .d_m = 11.0, }.Lane() );
}

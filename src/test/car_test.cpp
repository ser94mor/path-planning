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
      .trajectory_layer_queue_len = 9,
      .num_lanes = 10,
      .lane_width_m = 11,
      .max_s_m = 12,
      .behavior_planning_time_horizon_s = 13,
      .front_car_buffer_m = 5.0,
      .back_car_buffer_m = 0.5,
      .side_car_buffer_m = 1.0,
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
                                2.0,             /*x*/
                                1.0,             /*y*/
                                1.0,             /*vx*/
                                -1.0,            /*vy*/
                                1.5*sqrt(2.0),   /*s*/
                                0.5*sqrt(2.0),   /*d*/
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

TEST_CASE("GetFrenetCarsInLane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 4.0, };
  FrenetCar::SetPathPlannerConfig(&pp_config);

  std::vector<FrenetCar> all_cars(5);
  all_cars[0].d_m = 3;
  all_cars[1].d_m = 6;
  all_cars[2].d_m = 9;
  all_cars[3].d_m = 4.5;
  all_cars[4].d_m = 7.5;

  std::vector<FrenetCar> expected_cars_in_lane{ all_cars[1], all_cars[3], all_cars[4] };
  REQUIRE(GetFrenetCarsInLane(1, 4, all_cars) == expected_cars_in_lane );

  REQUIRE(GetFrenetCarsInLane(3, 4, all_cars).empty() );

  expected_cars_in_lane = { all_cars[0] };
  REQUIRE(GetFrenetCarsInLane(0, 4, all_cars) == expected_cars_in_lane );

  expected_cars_in_lane = { all_cars[2] };
  REQUIRE(GetFrenetCarsInLane(2, 4, all_cars) == expected_cars_in_lane );
}

TEST_CASE("GetNearestFrenetCarAheadBySCoordIfPresent", "car")
{
  FrenetCar ego_car{ .s_m = 1.0, };

  auto res = GetNearestFrenetCarAheadBySCoordIfPresent(ego_car, {});
  REQUIRE( !res.has_value() );

  FrenetCar other_car1{ .s_m = 2.0, };
  res = GetNearestFrenetCarAheadBySCoordIfPresent(ego_car, { other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  FrenetCar other_car2{ .s_m = 3.0, };
  res = GetNearestFrenetCarAheadBySCoordIfPresent(ego_car, { other_car1, other_car2 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  res = GetNearestFrenetCarAheadBySCoordIfPresent(ego_car, { other_car2, other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  FrenetCar other_car3{ .s_m = 0.5, };
  res = GetNearestFrenetCarAheadBySCoordIfPresent(ego_car, { other_car3, other_car2, other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );
}

TEST_CASE("FrenetCar::LongitudinalForwardDistanceTo", "[car]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);

  FrenetCar ego_car{ .s_m = 0.0, };
  FrenetCar other_car{ .s_m = 2.9, };
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

TEST_CASE("FrenetCar::LongitudinalBackwardDistanceTo", "[car]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);

  FrenetCar ego_car{ .s_m = 0.0, };
  FrenetCar other_car{ .s_m = 2.9, };
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

TEST_CASE("FrenetCar::LateralDistanceTo", "[car]")
{
  FrenetCar ego_car{ .d_m = 1.0, };
  FrenetCar other_car{ .d_m = 1.0, };

  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.0) );

  other_car.d_m = 0.5;
  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.5) );

  other_car.d_m = 1.5;
  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.5) );
}

TEST_CASE("FrenetCar::IsFrontBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config{ .max_s_m = 10.0, };
  FrenetCar::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  FrenetCar ego_car{ .s_m = 1.0, };
  FrenetCar other_car{ .s_m = 2.0, };
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

TEST_CASE("FrenetCar::IsBackBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config{ .max_s_m = 10.0, };
  FrenetCar::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  FrenetCar ego_car{ .s_m = 2.0, };
  FrenetCar other_car{ .s_m = 1.0, };
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

TEST_CASE("FrenetCar::IsSideBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config;
  FrenetCar::SetPathPlannerConfig(&pp_config);

  FrenetCar ego_car{ .d_m = 1.0, };
  FrenetCar other_car{ .d_m = 2.0, };
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

TEST_CASE("FrenetCar::Lane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 3.0 };
  FrenetCar::SetPathPlannerConfig(&pp_config);

  REQUIRE( -3 == FrenetCar{ .d_m = -8.3, }.Lane() );
  REQUIRE( -2 == FrenetCar{ .d_m = -4.0, }.Lane() );
  REQUIRE( -1 == FrenetCar{ .d_m = -0.1, }.Lane() );
  REQUIRE(  0 == FrenetCar{ .d_m =  2.3, }.Lane() );
  REQUIRE(  1 == FrenetCar{ .d_m =  3.1, }.Lane() );
  REQUIRE(  2 == FrenetCar{ .d_m =  7.5, }.Lane() );
  REQUIRE(  3 == FrenetCar{ .d_m = 11.0, }.Lane() );
}

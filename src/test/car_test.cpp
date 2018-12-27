//
// Created by aoool on 28.10.18.
//

#include "car.hpp"

#include <catch.hpp>
#include <ostream>


TEST_CASE("operator<<(Car)", "[car]") {
  std::ostringstream oss;

  Car car{
    Car::Builder()
      .SetId(-1)
      .SetState(FSM::State::PrepareLaneChangeRight)
      .SetTime(2.2)
      .SetCoordinateS(9.9)
      .SetCoordinateD(10.10)
      .SetVelocityS(11.11)
      .SetVelocityD(12.12)
      .SetAccelerationS(13.13)
      .SetAccelerationD(14.14)
    .Build()
  };

  std::string expected_res =
      "Car{\n"
      "  .id         = -1,\n"
      "  .state      = PLCR,\n"
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

  REQUIRE( car.Id()    == 12 );
  REQUIRE( car.State() == FSM::State::KeepLane );
  REQUIRE( car.T()     == Approx(2.22) );
  REQUIRE( car.S()     == Approx(1.5*sqrt(2.0)) );
  REQUIRE( car.D()     == Approx(-0.5*sqrt(2.0)) );
  REQUIRE( car.Vs()    == Approx(sqrt(2.0)) );
  REQUIRE( car.Vd()    == Approx(0.0) );
  REQUIRE( car.As()    == Approx(0.0) );
  REQUIRE( car.Ad()    == Approx(0.0) );
}


TEST_CASE("Car::V", "[car]")
{
  REQUIRE( Car::Builder(Car{}).SetVelocityS(2.0).SetVelocityD(5.0).Build().V() == Approx(sqrt(29.0)) );
  REQUIRE( Car::Builder(Car{}).SetVelocityS(5.0).SetVelocityD(2.0).Build().V() == Approx(sqrt(29.0)) );
}


TEST_CASE("Car::A", "[car]")
{
  REQUIRE( Car::Builder(Car{}).SetAccelerationS(2.0).SetAccelerationD(5.0).Build().A() == Approx(sqrt(29.0)) );
  REQUIRE( Car::Builder(Car{}).SetAccelerationS(5.0).SetAccelerationD(2.0).Build().A() == Approx(sqrt(29.0)) );
}


TEST_CASE("Car::LongitudinalForwardDistanceTo", "[car]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);

  Car ego_car{Car::Builder(Car{}).SetCoordinateS(0.0).Build() };
  Car other_car{Car::Builder(Car{}).SetCoordinateS(2.9).Build() };
  REQUIRE( ego_car.LongitudinalForwardDistanceTo(other_car) == Approx(2.9) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(2.9).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(0.0).Build();
  REQUIRE( ego_car.LongitudinalForwardDistanceTo(other_car) == Approx(0.1) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(2.9).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(0.1).Build();
  REQUIRE( ego_car.LongitudinalForwardDistanceTo(other_car) == Approx(0.2) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(0.1).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(2.9).Build();
  REQUIRE( ego_car.LongitudinalForwardDistanceTo(other_car) == Approx(2.8) );
}


TEST_CASE("Car::LongitudinalBackwardDistanceTo", "[car]")
{
  circular_unsigned_double_t::SetGlobalMaxValue(3.0);

  Car ego_car{Car::Builder(Car{}).SetCoordinateS(0.0).Build() };
  Car other_car{Car::Builder(Car{}).SetCoordinateS(2.9).Build() };
  REQUIRE( ego_car.LongitudinalBackwardDistanceTo(other_car) == Approx(0.1) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(2.9).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(0.0).Build();
  REQUIRE( ego_car.LongitudinalBackwardDistanceTo(other_car) == Approx(2.9) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(2.9).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(0.1).Build();
  REQUIRE( ego_car.LongitudinalBackwardDistanceTo(other_car) == Approx(2.8) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(0.1).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(2.9).Build();
  REQUIRE( ego_car.LongitudinalBackwardDistanceTo(other_car) == Approx(0.2) );
}


TEST_CASE("Car::LateralDistanceTo", "[car]")
{
  Car ego_car{Car::Builder(Car{}).SetCoordinateD(1.0).Build() };
  Car other_car{Car::Builder(Car{}).SetCoordinateD(1.0).Build() };
  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.0) );

  other_car = Car::Builder(other_car).SetCoordinateD(0.5).Build();
  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.5) );

  other_car = Car::Builder(other_car).SetCoordinateD(1.5).Build();
  REQUIRE(ego_car.LateralDistanceTo(other_car) == Approx(0.5) );
}


TEST_CASE("Car::IsFrontBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config{ .max_s_m = 10.0, };
  Car::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  Car ego_car{Car::Builder(Car{}).SetCoordinateS(1.0).Build() };
  Car other_car{Car::Builder(Car{}).SetCoordinateS(2.0).Build() };

  pp_config.front_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsFrontBufferViolatedBy(other_car) );

  pp_config.front_car_buffer_m = 0.5;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(9.1).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(0.0).Build();
  pp_config.front_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(9.1).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(0.1).Build();
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(9.0).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(0.0).Build();
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(10.0).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(1.0).Build();
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(10.5).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(1.5).Build();
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsFrontBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(10.5).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(1.0).Build();
  pp_config.front_car_buffer_m = 1.0;
  REQUIRE( ego_car.IsFrontBufferViolatedBy(other_car) );
}


TEST_CASE("Car::IsBackBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config{ .max_s_m = 10.0, };
  Car::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  Car ego_car{Car::Builder(Car{}).SetCoordinateS(2.0).Build() };
  Car other_car{Car::Builder(Car{}).SetCoordinateS(1.0).Build() };
  pp_config.back_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsBackBufferViolatedBy(other_car) );

  pp_config.back_car_buffer_m = 0.5;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(0.0).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(9.1).Build();
  pp_config.back_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(0.1).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(9.1).Build();
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(0.0).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(9.0).Build();
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(1.0).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(10.0).Build();
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(1.5).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(10.5).Build();
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsBackBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateS(1.0).Build();
  other_car = Car::Builder(other_car).SetCoordinateS(10.5).Build();
  pp_config.back_car_buffer_m = 1.0;
  REQUIRE( ego_car.IsBackBufferViolatedBy(other_car) );
}


TEST_CASE("Car::IsSideBufferViolatedBy", "[car]")
{
  PathPlannerConfig pp_config;
  Car::SetPathPlannerConfig(&pp_config);

  Car ego_car{Car::Builder(Car{}).SetCoordinateD(1.0).Build() };
  Car other_car{Car::Builder(Car{}).SetCoordinateD(2.0).Build() };
  pp_config.side_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsSideBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateD(1.0).Build();
  other_car = Car::Builder(other_car).SetCoordinateD(2.0).Build();
  pp_config.side_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsSideBufferViolatedBy(other_car) );

  ego_car = Car::Builder(ego_car).SetCoordinateD(3.0).Build();
  other_car = Car::Builder(other_car).SetCoordinateD(2.0).Build();
  pp_config.side_car_buffer_m = 1.0;
  REQUIRE( !ego_car.IsSideBufferViolatedBy(other_car) );

  pp_config.side_car_buffer_m = 1.1;
  REQUIRE( ego_car.IsSideBufferViolatedBy(other_car) );
}


TEST_CASE("Car::CurrentLane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 3.0 };
  Car::SetPathPlannerConfig(&pp_config);

  REQUIRE( -3 == Car::Builder(Car{}).SetCoordinateD(-8.3).Build().CurrentLane() );
  REQUIRE( -2 == Car::Builder(Car{}).SetCoordinateD(-4.0).Build().CurrentLane() );
  REQUIRE( -1 == Car::Builder(Car{}).SetCoordinateD(-0.1).Build().CurrentLane() );
  REQUIRE(  0 == Car::Builder(Car{}).SetCoordinateD(2.3).Build().CurrentLane()  );
  REQUIRE(  1 == Car::Builder(Car{}).SetCoordinateD(3.1).Build().CurrentLane()  );
  REQUIRE(  2 == Car::Builder(Car{}).SetCoordinateD(7.5).Build().CurrentLane()  );
  REQUIRE(  3 == Car::Builder(Car{}).SetCoordinateD(11.0).Build().CurrentLane() );
}


TEST_CASE("Car::IntendedLane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 4.0 };

  Car kl_car{Car::Builder(Car{}).SetState(FSM::State::KeepLane).Build()};
  REQUIRE(Car::Builder(kl_car).SetCoordinateD(2.0).Build().IntendedLane() == 0 );
  REQUIRE(Car::Builder(kl_car).SetCoordinateD(6.0).Build().IntendedLane()  == 1 );
  REQUIRE(Car::Builder(kl_car).SetCoordinateD(10.0).Build().IntendedLane() == 2 );

  Car lcl_car{Car::Builder(
      Car::Builder(kl_car).SetState(FSM::State::PrepareLaneChangeLeft).Build())
                  .SetState(FSM::State::LaneChangeLeft)
                  .Build()};
  REQUIRE(Car::Builder(lcl_car).SetState(FSM::State::KeepLane).SetCoordinateD(2.0).Build().IntendedLane() == 0 );
  REQUIRE(Car::Builder(lcl_car).SetState(FSM::State::KeepLane).SetCoordinateD(6.0).Build().IntendedLane()  == 1 );
  REQUIRE(Car::Builder(lcl_car).SetState(FSM::State::KeepLane).SetCoordinateD(10.0).Build().IntendedLane() == 2 );

  Car lcr_car{Car::Builder(
      Car::Builder(kl_car).SetState(FSM::State::PrepareLaneChangeRight).Build())
                  .SetState(FSM::State::LaneChangeRight)
                  .Build()};
  REQUIRE(Car::Builder(lcr_car).SetState(FSM::State::KeepLane).SetCoordinateD(2.0).Build().IntendedLane() == 0 );
  REQUIRE(Car::Builder(lcr_car).SetState(FSM::State::KeepLane).SetCoordinateD(6.0).Build().IntendedLane()  == 1 );
  REQUIRE(Car::Builder(lcr_car).SetState(FSM::State::KeepLane).SetCoordinateD(10.0).Build().IntendedLane() == 2 );


  Car plcl_car{Car::Builder(Car{}).SetState(FSM::State::PrepareLaneChangeLeft).Build()};
  REQUIRE(Car::Builder(plcl_car).SetCoordinateD(2.0).Build().IntendedLane()  == -1 );
  REQUIRE(Car::Builder(plcl_car).SetCoordinateD(6.0).Build().IntendedLane()  ==  0 );
  REQUIRE(Car::Builder(plcl_car).SetCoordinateD(10.0).Build().IntendedLane() ==  1 );

  lcl_car = Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).Build();
  REQUIRE(Car::Builder(lcl_car).SetCoordinateD(2.0).Build().IntendedLane()  == -1 );
  REQUIRE(Car::Builder(lcl_car).SetCoordinateD(6.0).Build().IntendedLane()  == -1 );
  REQUIRE(Car::Builder(lcl_car).SetCoordinateD(10.0).Build().IntendedLane() == -1 );

  plcl_car = Car::Builder(plcl_car).SetCoordinateD(2.0).Build();
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).Build().IntendedLane()  == -1 );
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).SetCoordinateD(-0.5).Build().IntendedLane() == -1 );
  plcl_car = Car::Builder(Car::Builder(plcl_car).SetCoordinateD(6.0).Build()).Build();
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).Build().IntendedLane()  == 0 );
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).SetCoordinateD(2.0).Build().IntendedLane()  == 0 );
  plcl_car = Car::Builder(Car::Builder(plcl_car).SetCoordinateD(10.0).Build()).Build();
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).Build().IntendedLane() == 1 );
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).SetCoordinateD(6.0).Build().IntendedLane() == 1 );

  Car plcr_car{Car::Builder(Car{}).SetState(FSM::State::PrepareLaneChangeRight).Build()};
  REQUIRE(Car::Builder(plcr_car).SetCoordinateD(2.0).Build().IntendedLane()  == 1 );
  REQUIRE(Car::Builder(plcr_car).SetCoordinateD(6.0).Build().IntendedLane()  == 2 );
  REQUIRE(Car::Builder(plcr_car).SetCoordinateD(10.0).Build().IntendedLane() == 3 );

  lcr_car = Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).Build();
  REQUIRE(Car::Builder(lcr_car).SetCoordinateD(2.0).Build().IntendedLane()  == 1 );
  REQUIRE(Car::Builder(lcr_car).SetCoordinateD(6.0).Build().IntendedLane()  == 1 );
  REQUIRE(Car::Builder(lcr_car).SetCoordinateD(10.0).Build().IntendedLane() == 1 );


  plcr_car = Car::Builder(plcr_car).SetCoordinateD(2.0).Build();
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).Build().IntendedLane()  == 1 );
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).SetCoordinateD(6.0).Build().IntendedLane()  == 1 );
  plcr_car = Car::Builder(plcr_car).SetCoordinateD(6.0).Build();
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).Build().IntendedLane()  == 2 );
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).SetCoordinateD(10.0).Build().IntendedLane()  == 2 );
  plcr_car = Car::Builder(plcr_car).SetCoordinateD(10.0).Build();
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).Build().IntendedLane() == 3 );
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).SetCoordinateD(14.0).Build().IntendedLane() == 3 );
}


TEST_CASE("Car::FinalLane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 4.0 };

  Car kl_car{Car::Builder(Car{}).SetState(FSM::State::KeepLane).Build()};
  REQUIRE(Car::Builder(kl_car).SetCoordinateD(2.0).Build().FinalLane() == 0 );
  REQUIRE(Car::Builder(kl_car).SetCoordinateD(6.0).Build().FinalLane()  == 1 );
  REQUIRE(Car::Builder(kl_car).SetCoordinateD(10.0).Build().FinalLane() == 2 );

  Car lcl_car{Car::Builder(
      Car::Builder(kl_car).SetState(FSM::State::PrepareLaneChangeLeft).Build())
                  .SetState(FSM::State::LaneChangeLeft)
                  .Build()};
  REQUIRE(Car::Builder(lcl_car).SetState(FSM::State::KeepLane).SetCoordinateD(2.0).Build().FinalLane() == 0 );
  REQUIRE(Car::Builder(lcl_car).SetState(FSM::State::KeepLane).SetCoordinateD(6.0).Build().FinalLane()  == 1 );
  REQUIRE(Car::Builder(lcl_car).SetState(FSM::State::KeepLane).SetCoordinateD(10.0).Build().FinalLane() == 2 );

  Car lcr_car{Car::Builder(
      Car::Builder(kl_car).SetState(FSM::State::PrepareLaneChangeRight).Build())
                  .SetState(FSM::State::LaneChangeRight)
                  .Build()};
  REQUIRE(Car::Builder(lcr_car).SetState(FSM::State::KeepLane).SetCoordinateD(2.0).Build().FinalLane() == 0 );
  REQUIRE(Car::Builder(lcr_car).SetState(FSM::State::KeepLane).SetCoordinateD(6.0).Build().FinalLane()  == 1 );
  REQUIRE(Car::Builder(lcr_car).SetState(FSM::State::KeepLane).SetCoordinateD(10.0).Build().FinalLane() == 2 );



  Car plcl_car{Car::Builder(Car{}).SetState(FSM::State::PrepareLaneChangeLeft).Build()};
  REQUIRE(Car::Builder(plcl_car).SetCoordinateD(2.0).Build().FinalLane()  == 0 );
  REQUIRE(Car::Builder(plcl_car).SetCoordinateD(6.0).Build().FinalLane()  == 1 );
  REQUIRE(Car::Builder(plcl_car).SetCoordinateD(10.0).Build().FinalLane() == 2 );

  lcl_car = Car::Builder(Car{}).SetState(FSM::State::LaneChangeLeft).Build();
  REQUIRE(Car::Builder(lcl_car).SetCoordinateD(2.0).Build().FinalLane()  == -1 );
  REQUIRE(Car::Builder(lcl_car).SetCoordinateD(6.0).Build().FinalLane()  == -1 );
  REQUIRE(Car::Builder(lcl_car).SetCoordinateD(10.0).Build().FinalLane() == -1 );

  plcl_car = Car::Builder(plcl_car).SetCoordinateD(2.0).Build();
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).Build().FinalLane()  == -1 );
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).SetCoordinateD(-0.5).Build().FinalLane() == -1 );
  plcl_car = Car::Builder(Car::Builder(plcl_car).SetCoordinateD(6.0).Build()).Build();
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).Build().FinalLane()  == 0 );
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).SetCoordinateD(2.0).Build().FinalLane()  == 0 );
  plcl_car = Car::Builder(Car::Builder(plcl_car).SetCoordinateD(10.0).Build()).Build();
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).Build().FinalLane() == 1 );
  REQUIRE(Car::Builder(plcl_car).SetState(FSM::State::LaneChangeLeft).SetCoordinateD(6.0).Build().FinalLane() == 1 );

  Car plcr_car{Car::Builder(Car{}).SetState(FSM::State::PrepareLaneChangeRight).Build()};
  REQUIRE(Car::Builder(plcr_car).SetCoordinateD(2.0).Build().FinalLane()  == 0 );
  REQUIRE(Car::Builder(plcr_car).SetCoordinateD(6.0).Build().FinalLane()  == 1 );
  REQUIRE(Car::Builder(plcr_car).SetCoordinateD(10.0).Build().FinalLane() == 2 );

  lcr_car = Car::Builder(Car{}).SetState(FSM::State::LaneChangeRight).Build();
  REQUIRE(Car::Builder(lcr_car).SetCoordinateD(2.0).Build().FinalLane()  == 1 );
  REQUIRE(Car::Builder(lcr_car).SetCoordinateD(6.0).Build().FinalLane()  == 1 );
  REQUIRE(Car::Builder(lcr_car).SetCoordinateD(10.0).Build().FinalLane() == 1 );

  plcr_car = Car::Builder(plcr_car).SetCoordinateD(2.0).Build();
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).Build().FinalLane()  == 1 );
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).SetCoordinateD(6.0).Build().FinalLane()  == 1 );
  plcr_car = Car::Builder(plcr_car).SetCoordinateD(6.0).Build();
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).Build().FinalLane()  == 2 );
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).SetCoordinateD(10.0).Build().FinalLane()  == 2 );
  plcr_car = Car::Builder(plcr_car).SetCoordinateD(10.0).Build();
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).Build().FinalLane() == 3 );
  REQUIRE(Car::Builder(plcr_car).SetState(FSM::State::LaneChangeRight).SetCoordinateD(14.0).Build().FinalLane() == 3 );
}


TEST_CASE("Car::CarsInRegionOfInterest", "[car]")
{
  PathPlannerConfig pp_config{
    .max_s_m = 100.0,
    .region_of_interest_front_m = 20.0,
    .region_of_interest_back_m = 10.0,
  };
  Car::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  Car ego_car{Car::Builder(Car{}).SetCoordinateS(50.0).Build() };
  Car car1{Car::Builder(Car{}).SetCoordinateS(60.0).Build() };
  Car car2{Car::Builder(Car{}).SetCoordinateS(70.0).Build()  };
  REQUIRE( ego_car.CarsInRegionOfInterest({ car1, car2, }) == std::vector{ car1, } );

  ego_car = Car::Builder(ego_car).SetCoordinateS(50.0).Build();
  car1 = Car::Builder(car1).SetCoordinateS(50.0).Build();
  car2 = Car::Builder(car2).SetCoordinateS(69.0).Build();
  REQUIRE( ego_car.CarsInRegionOfInterest({ car2, car1, }) == std::vector{ car2, car1, } );

  Car car3{Car::Builder(Car{}).SetCoordinateS(40.0).Build()  };
  REQUIRE( ego_car.CarsInRegionOfInterest({ car1, car2, car3, }) == std::vector{ car1, car2, car3, } );

  car3 = Car::Builder(car3).SetCoordinateS(39.0).Build();
  REQUIRE( ego_car.CarsInRegionOfInterest({ car1, car2, car3, }) == std::vector{ car1, car2, } );

  ego_car = Car::Builder(ego_car).SetCoordinateS(1.0).Build();
  car1 = Car::Builder(car1).SetCoordinateS(10.0).Build();
  car2 = Car::Builder(car2).SetCoordinateS(30.0).Build();
  car3 = Car::Builder(car3).SetCoordinateS(95.0).Build();
  REQUIRE( ego_car.CarsInRegionOfInterest({ car1, car2, car3, }) == std::vector{ car1, car3, } );

  ego_car = Car::Builder(ego_car).SetCoordinateS(90.0).Build();
  car1 = Car::Builder(car1).SetCoordinateS(9.0).Build();
  car2 = Car::Builder(car2).SetCoordinateS(30.0).Build();
  car3 = Car::Builder(car3).SetCoordinateS(80.0).Build();
  REQUIRE( ego_car.CarsInRegionOfInterest({ car1, car2, car3, }) == std::vector{ car1, car3, } );
}


TEST_CASE("Car::CarsInCurrentLane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 3.0 };
  Car::SetPathPlannerConfig(&pp_config);

  std::vector<Car> cars(6);
  for (int i = 0; i < cars.size(); ++i) {
    cars[i] = Car::Builder(cars[i]).SetCoordinateD(0.1 + 1.5 * i).Build();
  }

  Car ego_car{Car::Builder(Car{}).SetCoordinateD(4.5).Build() };
  REQUIRE( ego_car.CarsInCurrentLane(cars) == std::vector{ cars[2], cars[3] } );

  ego_car = Car::Builder(Car{}).SetCoordinateD(1.0).Build();
  REQUIRE( ego_car.CarsInCurrentLane(cars) == std::vector{ cars[0], cars[1] } );

  ego_car = Car::Builder(Car{}).SetCoordinateD(7.3).Build();
  REQUIRE( ego_car.CarsInCurrentLane(cars) == std::vector{ cars[4], cars[5] } );
}


TEST_CASE("Car::CarsInIntendedLane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 3.0 };
  Car::SetPathPlannerConfig(&pp_config);

  std::vector<Car> cars(6);
  for (int i = 0; i < cars.size(); ++i) {
    cars[i] = Car::Builder(cars[i]).SetCoordinateD(0.1 + 1.5 * i).Build();
  }

  Car ego_car{Car::Builder(Car{}).SetCoordinateD(4.5).Build() };
  REQUIRE( ego_car.CarsInIntendedLane(cars) == std::vector{ cars[2], cars[3] } );

  ego_car = Car::Builder(ego_car).SetState(FSM::State::PrepareLaneChangeLeft).Build();
  REQUIRE( ego_car.CarsInIntendedLane(cars) == std::vector{ cars[0], cars[1] } );
  ego_car = Car::Builder(ego_car).SetState(FSM::State::LaneChangeLeft).Build();
  REQUIRE( ego_car.CarsInIntendedLane(cars) == std::vector{ cars[0], cars[1] } );

  ego_car = Car::Builder(ego_car).SetState(FSM::State::PrepareLaneChangeRight).Build();
  REQUIRE( ego_car.CarsInIntendedLane(cars) == std::vector{ cars[4], cars[5] } );
  ego_car = Car::Builder(ego_car).SetState(FSM::State::LaneChangeRight).Build();
  REQUIRE( ego_car.CarsInIntendedLane(cars) == std::vector{ cars[4], cars[5] } );
}


TEST_CASE("Car::CarsInFinalLane", "[car]")
{
  PathPlannerConfig pp_config{ .lane_width_m = 3.0 };
  Car::SetPathPlannerConfig(&pp_config);

  std::vector<Car> cars(6);
  for (int i = 0; i < cars.size(); ++i) {
    cars[i] = Car::Builder(cars[i]).SetCoordinateD(0.1 + 1.5 * i).Build();
  }

  Car ego_car{Car::Builder(Car{}).SetCoordinateD(4.5).Build() };
  REQUIRE( ego_car.CarsInFinalLane(cars) == std::vector{ cars[2], cars[3] } );
  ego_car = Car::Builder(ego_car).SetState(FSM::State::PrepareLaneChangeLeft).Build();
  REQUIRE( ego_car.CarsInFinalLane(cars) == std::vector{ cars[2], cars[3] } );
  ego_car = Car::Builder(ego_car).SetState(FSM::State::PrepareLaneChangeRight).Build();
  REQUIRE( ego_car.CarsInFinalLane(cars) == std::vector{ cars[2], cars[3] } );

  ego_car = Car::Builder(ego_car).SetState(FSM::State::LaneChangeLeft).Build();
  REQUIRE( ego_car.CarsInFinalLane(cars) == std::vector{ cars[0], cars[1] } );

  ego_car = Car::Builder(ego_car).SetState(FSM::State::LaneChangeRight).Build();
  REQUIRE( ego_car.CarsInFinalLane(cars) == std::vector{ cars[4], cars[5] } );
}


TEST_CASE("Car::NearestCarAhead", "car")
{
  Car ego_car{Car::Builder(Car{}).SetCoordinateS(1.0).Build() };

  auto res = ego_car.NearestCarAhead({});
  REQUIRE( !res.has_value() );

  Car other_car1{Car::Builder(Car{}).SetCoordinateS(2.0).Build() };
  res = ego_car.NearestCarAhead({ other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  Car other_car2{Car::Builder(Car{}).SetCoordinateS(3.0).Build() };
  res = ego_car.NearestCarAhead({ other_car1, other_car2 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  res = ego_car.NearestCarAhead({ other_car2, other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  Car other_car3{Car::Builder(Car{}).SetCoordinateS(0.5).Build() };
  res = ego_car.NearestCarAhead({ other_car3, other_car2, other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );
}


TEST_CASE("Car::NearestCarBehind", "car")
{
  Car ego_car{Car::Builder(Car{}).SetCoordinateS(3.0).Build() };

  auto res = ego_car.NearestCarBehind({});
  REQUIRE( !res.has_value() );

  Car other_car1{Car::Builder(Car{}).SetCoordinateS(2.0).Build() };
  res = ego_car.NearestCarBehind({ other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  Car other_car2{Car::Builder(Car{}).SetCoordinateS(1.0).Build() };
  res = ego_car.NearestCarBehind({ other_car1, other_car2 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  res = ego_car.NearestCarBehind({ other_car2, other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car1) );

  Car other_car3{Car::Builder(Car{}).SetCoordinateS(2.5).Build() };
  res = ego_car.NearestCarBehind({ other_car3, other_car2, other_car1 });
  REQUIRE( (res.has_value() && res.value() == other_car3) );
}


TEST_CASE("Car::IsInFrontOf", "[car]")
{
  PathPlannerConfig pp_config{
    .max_s_m = 10.0,
  };
  Car::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  Car ego_car{Car::Builder(Car{}).SetCoordinateS(1.0).Build() };
  Car other_car{Car::Builder(Car{}).SetCoordinateS(4.0).Build() };
  REQUIRE( !ego_car.IsInFrontOf(other_car) );

  other_car = Car::Builder(other_car).SetCoordinateS(6.0).Build();
  REQUIRE( !ego_car.IsInFrontOf(other_car) );

  other_car = Car::Builder(other_car).SetCoordinateS(6.1).Build();;
  REQUIRE( ego_car.IsInFrontOf(other_car) );

  other_car = Car::Builder(other_car).SetCoordinateS(9.0).Build();;
  REQUIRE( ego_car.IsInFrontOf(other_car) );

  other_car = Car::Builder(other_car).SetCoordinateS(0.1).Build();;
  REQUIRE( ego_car.IsInFrontOf(other_car) );
}

TEST_CASE("Car::IsBehind", "[car]")
{
  PathPlannerConfig pp_config{
      .max_s_m = 10.0,
  };
  Car::SetPathPlannerConfig(&pp_config);
  circular_unsigned_double_t::SetGlobalMaxValue(pp_config.max_s_m);

  Car ego_car{Car::Builder(Car{}).SetCoordinateS(1.0).Build() };
  Car other_car{Car::Builder(Car{}).SetCoordinateS(4.0).Build() };
  REQUIRE( ego_car.IsBehind(other_car) );

  other_car = Car::Builder(other_car).SetCoordinateS(6.0).Build();
  REQUIRE( ego_car.IsBehind(other_car) );

  other_car = Car::Builder(other_car).SetCoordinateS(6.1).Build();
  REQUIRE( !ego_car.IsBehind(other_car) );

  other_car = Car::Builder(other_car).SetCoordinateS(9.0).Build();
  REQUIRE( !ego_car.IsBehind(other_car) );

  other_car = Car::Builder(other_car).SetCoordinateS(0.1).Build();
  REQUIRE( !ego_car.IsBehind(other_car) );
}




TEST_CASE() {
  std::map<Car, Car> cars{ { Car::Builder()
                               .SetId(1)
                               .SetState(FSM::State::KeepLane)
                               .SetTime(2.0)
                               .SetCoordinateS(1.0)
                               .SetCoordinateD(0.5)
                               .SetVelocityS(30.0)
                               .SetVelocityD(0.1)
                               .SetAccelerationS(0.45)
                               .SetAccelerationD(0.0)
                             .Build(),
                             Car::Builder()
                               .SetId(1)
                               .SetState(FSM::State::KeepLane)
                               .SetTime(4.0)
                               .SetCoordinateS(5.0)
                               .SetCoordinateD(0.7)
                               .SetVelocityS(40.0)
                               .SetVelocityD(0.7)
                               .SetAccelerationS(0.0)
                               .SetAccelerationD(0.38)
                             .Build(), },
                           { Car::Builder()
                               .SetId(2)
                               .SetState(FSM::State::LaneChangeRight)
                               .SetTime(2.0)
                               .SetCoordinateS(1.0)
                               .SetCoordinateD(0.5)
                               .SetVelocityS(34.0)
                               .SetVelocityD(0.7)
                               .SetAccelerationS(10.0)
                               .SetAccelerationD(7.4)
                             .Build(),
                             Car::Builder()
                               .SetId(2)
                               .SetState(FSM::State::LaneChangeLeft)
                               .SetTime(4.0)
                               .SetCoordinateS(67.0)
                               .SetCoordinateD(3.0)
                               .SetVelocityS(70.0)
                               .SetVelocityD(0.0)
                               .SetAccelerationS(0.01)
                               .SetAccelerationD(0.02)
                             .Build(), },
                         };

  std::string expected{"Car{                          Car{\n"
                       "  .id         = 1,              .id         = 1,\n"
                       "  .state      = KL,             .state      = KL,\n"
                       "  .time_s     = 2.000000,       .time_s     = 4.000000,\n"
                       "  .s_m        = 1.000000,       .s_m        = 5.000000,\n"
                       "  .d_m        = 0.500000,       .d_m        = 0.700000,\n"
                       "  .vel_s_mps  = 30.000000,      .vel_s_mps  = 40.000000,\n"
                       "  .vel_d_mps  = 0.100000,       .vel_d_mps  = 0.700000,\n"
                       "  .acc_s_mps2 = 0.450000,       .acc_s_mps2 = 0.000000,\n"
                       "  .acc_d_mps2 = 0.000000,       .acc_d_mps2 = 0.380000,\n"
                       "}                             }\n"
                       "\n"
                       "Car{                          Car{\n"
                       "  .id         = 2,              .id         = 2,\n"
                       "  .state      = LCR,            .state      = LCL,\n"
                       "  .time_s     = 2.000000,       .time_s     = 4.000000,\n"
                       "  .s_m        = 1.000000,       .s_m        = 7.000000,\n"
                       "  .d_m        = 0.500000,       .d_m        = 3.000000,\n"
                       "  .vel_s_mps  = 34.000000,      .vel_s_mps  = 70.000000,\n"
                       "  .vel_d_mps  = 0.700000,       .vel_d_mps  = 0.000000,\n"
                       "  .acc_s_mps2 = 10.000000,      .acc_s_mps2 = 0.010000,\n"
                       "  .acc_d_mps2 = 7.400000,       .acc_d_mps2 = 0.020000,\n"
                       "}                             }"};

  REQUIRE(expected == Car::CarMapToString(cars));
}


TEST_CASE("Car::TimeSinceLastManeuver", "[car]")
{
  Car car{Car::Builder(Car{}).SetState(FSM::State::KeepLane).SetTime(0.0).Build()};
  REQUIRE( car.TimeSinceLastManeuver() == Approx(0.0) );

  car = Car::Builder(car).SetTime(3.5).Build();
  REQUIRE( car.TimeSinceLastManeuver() == Approx(3.5) );

  car = Car::Builder(car).SetState(FSM::State::LaneChangeLeft).SetTime(4.0).Build();
  REQUIRE( car.TimeSinceLastManeuver() == Approx(4.0) );

  car = Car::Builder(car).SetState(FSM::State::KeepLane).SetTime(5.0).Build();
  REQUIRE( car.TimeSinceLastManeuver() == Approx(0.0) );

  car = Car::Builder(car).SetState(FSM::State::LaneChangeRight).SetTime(5.5).Build();
  REQUIRE( car.TimeSinceLastManeuver() == Approx(0.5) );

  car = Car::Builder(car).SetState(FSM::State::LaneChangeRight).SetTime(6.0).Build();
  REQUIRE( car.TimeSinceLastManeuver() == Approx(1.0) );

  car = Car::Builder(car).SetState(FSM::State::KeepLane).SetTime(7.0).Build();
  REQUIRE( car.TimeSinceLastManeuver() == Approx(0.0) );

  car = Car::Builder(car).SetState(FSM::State::KeepLane).SetTime(8.0).Build();
  REQUIRE( car.TimeSinceLastManeuver() == Approx(1.0) );
}

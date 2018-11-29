//
// Created by aoool on 8/22/18.
//
#include <iostream>
#include "catch.hpp"

#include "helpers.hpp"


TEST_CASE("GetXY", "[helpers]")
{
  PathPlannerConfig config = {
      .frequency_s = 0.020,
      .min_speed_mps = 0.000,
      .max_speed_mps = 21.905,
      .min_acc_mps2 = -10.000,
      .max_acc_mps2 = 10.000,
      .min_jerk_mps3 = -10.000,
      .max_jerk_mps3 = 10.000,
      .path_len = 50,
      .num_lanes = 3,
      .lane_width_m = 1.000,
      .max_s_m = 4*pi(),
      .behavior_planning_time_horizon_s = 3,

      // circle with the center (0,0) and the radius 2
      // angle             0           pi/8      2pi/8          3pi/8 4pi/8           5pi/8         6pi/8           7pi/8        pi           9pi/8        10pi/8          11pi/8    12pi/8          13pi/8        14pi/8          15pi/8
      .map_wps_x_m  = {  2.0, 4.0/sqrt(5.0), sqrt(2.0), 2.0/sqrt(5.0),  0.0, -2.0/sqrt(5.0),   -sqrt(2.0), -4.0/sqrt(5.0),     -2.0, -4.0/sqrt(5.0),   -sqrt(2.0), -2.0/sqrt(5.0),      0.0,  2.0/sqrt(5.0),    sqrt(2.0),  4.0/sqrt(5.0), },
      .map_wps_y_m  = {  0.0, 2.0/sqrt(5.0), sqrt(2.0), 4.0/sqrt(5.0),  2.0,  4.0/sqrt(5.0),    sqrt(2.0),  2.0/sqrt(5.0),      0.0, -2.0/sqrt(5.0),   -sqrt(2.0), -4.0/sqrt(5.0),     -2.0, -4.0/sqrt(5.0),   -sqrt(2.0), -2.0/sqrt(5.0), },
      .map_wps_s_m  = {  0.0,      pi()/4.0,  pi()/2.0,  3.0*pi()/4.0, pi(),   5.0*pi()/4.0, 3.0*pi()/2.0,   7.0*pi()/4.0, 2.0*pi(),   9.0*pi()/4.0, 5.0*pi()/2.0,  11.0*pi()/4.0, 3.0*pi(),  13.0*pi()/4.0, 7.0*pi()/2.0,  15.0*pi()/4.0, },
      .map_wps_dx_m = {  1.0, 2.0/sqrt(5.0), sqrt(0.5), 1.0/sqrt(5.0),  0.0, -1.0/sqrt(5.0),   -sqrt(0.5), -2.0/sqrt(5.0),     -1.0, -2.0/sqrt(5.0),   -sqrt(0.5), -1.0/sqrt(5.0),      0.0,  1.0/sqrt(5.0),    sqrt(0.5),  2.0/sqrt(5.0), },
      .map_wps_dy_m = {  0.0, 1.0/sqrt(5.0), sqrt(0.5), 2.0/sqrt(5.0),  1.0,  2.0/sqrt(5.0),    sqrt(0.5),  1.0/sqrt(5.0),      0.0, -1.0/sqrt(5.0),   -sqrt(0.5), -2.0/sqrt(5.0),     -1.0, -2.0/sqrt(5.0),   -sqrt(0.5), -1.0/sqrt(5.0), },

      .spline_s_x  = {},
      .spline_s_y  = {},
      .spline_s_dx = {},
      .spline_s_dy = {},
  };
  config.InitSplines();

  SECTION("GetXY produces the same x and y coords for known points.")
  {
    auto coords = GetXY(0.0, 0.0, config);
    REQUIRE(coords[0] == Approx(2.0));
    REQUIRE( fabs(0.0 - coords[1]) < 0.00000001 );

    coords = GetXY(3.0*pi()/4.0, 0.0, config);
    REQUIRE(coords[0] == Approx(2.0/sqrt(5.0)));
    REQUIRE(coords[1] == Approx(4.0/sqrt(5.0)));

    coords = GetXY(3.0*pi()/2.0, 0.0, config);
    REQUIRE(coords[0] == Approx(-sqrt(2.0)));
    REQUIRE(coords[1] == Approx(sqrt(2.0)));
  }

  SECTION("GetXY produces approximate x and y values for unknown points.")
  {

    // 0.2 m == 20 sm
    double precision = 0.2;

    auto coords = GetXY(2.0*pi()/6.0, 1.0, config);
    REQUIRE( fabs(coords[0] - 3.0*sqrt(3.0)/2.0) < precision );
    REQUIRE( fabs(coords[1] - 1.5) < precision );

    coords = GetXY(2.0*7.0*pi()/6.0, 1.0, config);
    REQUIRE( fabs(coords[0] - (-3.0*sqrt(3.0)/2.0)) < precision );
    REQUIRE( fabs(coords[1] - (-1.5)) < precision );
  }
}


TEST_CASE("Calc1DSpeed", "[helpers]")
{
  REQUIRE( Calc1DSpeed(200.0, 100.0, 10.0) == Approx(-10.0) );
  REQUIRE( Calc1DSpeed(100.0, 100.0, 10.0) == Approx(0.0) );
  REQUIRE( Calc1DSpeed(100.0, 200.0, 10.0) == Approx(10.0) );
}

TEST_CASE("Calc1DAcc", "[helpers]") {
  REQUIRE( Calc1DAcc(800.0, 600.0, 200.0, 10.0) == Approx(-2.0) );
  REQUIRE( Calc1DAcc(400.0, 300.0, 100.0, 10.0) == Approx(-1.0) );
  REQUIRE( Calc1DAcc(100.0, 200.0, 300.0, 10.0) == Approx(0.0) );
  REQUIRE( Calc1DAcc(100.0, 200.0, 400.0, 10.0) == Approx(1.0) );
  REQUIRE( Calc1DAcc(200.0, 400.0, 800.0, 10.0) == Approx(2.0) );
}

TEST_CASE("Calc1DJerk", "[helpers]") {
  REQUIRE( Calc1DJerk(1000.0, 900.0, 700.0, 100.0, 10.0)  == Approx(-0.3) );
  REQUIRE( Calc1DJerk(900.0, 800.0, 600.0, 100.0, 10.0) == Approx(-0.2) );
  REQUIRE( Calc1DJerk(800.0, 700.0, 500.0, 100.0, 10.0) == Approx(-0.1) );
  REQUIRE( Calc1DJerk(100.0, 200.0, 400.0, 700.0, 10.0) == Approx(0.0) );
  REQUIRE( Calc1DJerk(100.0, 200.0, 400.0, 800.0, 10.0) == Approx(0.1) );
  REQUIRE( Calc1DJerk(100.0, 200.0, 400.0, 900.0, 10.0) == Approx(0.2) );
  REQUIRE( Calc1DJerk(100.0, 200.0, 400.0, 1000.0, 10.0) == Approx(0.3) );
}

TEST_CASE("CalcYawRad", "[helpers]") {
  REQUIRE( Approx(0.0)      == CalcYawRad(1.0, 0.0));
  REQUIRE( Approx(M_PI)     == CalcYawRad(-10.0, 0.0) );

  REQUIRE( Approx(M_PI_2)   == CalcYawRad(0.0, 4.0) );
  REQUIRE( Approx(3*M_PI_2) == CalcYawRad(0.0, -7.0) );

  REQUIRE( Approx(M_PI_4)   == CalcYawRad(16.5, 16.5) );
  REQUIRE( Approx(3*M_PI_4) == CalcYawRad(-11.567, 11.567) );
  REQUIRE( Approx(5*M_PI_4) == CalcYawRad(-5.0, -5.0) );
  REQUIRE( Approx(7*M_PI_4) == CalcYawRad(12.98, -12.98) );

  REQUIRE( Approx(M_PI / 6)      == CalcYawRad(sqrt(3), 1.0) );
  REQUIRE( Approx(M_PI / 3)      == CalcYawRad(1.0, sqrt(3)) );
  REQUIRE( Approx(2 * M_PI / 3)  == CalcYawRad(-1.0, sqrt(3)) );
  REQUIRE( Approx(11 * M_PI / 6) == CalcYawRad(sqrt(3), -1.0) );
  REQUIRE( Approx(5 * M_PI / 6)  == CalcYawRad(-sqrt(3), 1.0) );
  REQUIRE( Approx(5 * M_PI / 3)  == CalcYawRad(1.0, -sqrt(3)) );
  REQUIRE( Approx(7 * M_PI / 6)  == CalcYawRad(-sqrt(3), -1.0) );
  REQUIRE( Approx(4 * M_PI / 3)  == CalcYawRad(-1.0, -sqrt(3)) );
}

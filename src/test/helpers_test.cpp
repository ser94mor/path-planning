//
// Created by aoool on 8/22/18.
//
#include "catch.hpp"

#include "helpers.hpp"

TEST_CASE("calc_speed", "[helpers]") {
  REQUIRE( calc_speed(200.0, 100.0, 10.0) == Approx(-10.0) );
  REQUIRE( calc_speed(100.0, 100.0, 10.0) == Approx(0.0) );
  REQUIRE( calc_speed(100.0, 200.0, 10.0) == Approx(10.0) );
}

TEST_CASE("calc_acc", "[helpers]") {
  REQUIRE( calc_acc(800.0, 600.0, 200.0, 10.0) == Approx(-2.0) );
  REQUIRE( calc_acc(400.0, 300.0, 100.0, 10.0) == Approx(-1.0) );
  REQUIRE( calc_acc(100.0, 200.0, 300.0, 10.0) == Approx(0.0) );
  REQUIRE( calc_acc(100.0, 200.0, 400.0, 10.0) == Approx(1.0) );
  REQUIRE( calc_acc(200.0, 400.0, 800.0, 10.0) == Approx(2.0) );
}

TEST_CASE("calc_jerk", "[helpers]") {
  REQUIRE( calc_jerk(1000.0, 900.0, 700.0, 100.0, 10.0)  == Approx(-0.3) );
  REQUIRE( calc_jerk(900.0,  800.0, 600.0, 100.0,  10.0) == Approx(-0.2) );
  REQUIRE( calc_jerk(800.0,  700.0, 500.0, 100.0,  10.0) == Approx(-0.1) );
  REQUIRE( calc_jerk(100.0,  200.0, 400.0, 700.0,  10.0) == Approx(0.0) );
  REQUIRE( calc_jerk(100.0,  200.0, 400.0, 800.0,  10.0) == Approx(0.1) );
  REQUIRE( calc_jerk(100.0,  200.0, 400.0, 900.0,  10.0) == Approx(0.2) );
  REQUIRE( calc_jerk(100.0,  200.0, 400.0, 1000.0, 10.0) == Approx(0.3) );
}

TEST_CASE("calc_yaw_rad", "[helpers]") {
  REQUIRE( Approx(0.0)      == calc_yaw_rad(1.0, 0.0));
  REQUIRE( Approx(M_PI)     == calc_yaw_rad(-10.0, 0.0) );

  REQUIRE( Approx(M_PI_2)   == calc_yaw_rad(0.0, 4.0) );
  REQUIRE( Approx(3*M_PI_2) == calc_yaw_rad(0.0, -7.0) );

  REQUIRE( Approx(M_PI_4)   == calc_yaw_rad(16.5, 16.5) );
  REQUIRE( Approx(3*M_PI_4) == calc_yaw_rad(-11.567, 11.567) );
  REQUIRE( Approx(5*M_PI_4) == calc_yaw_rad(-5.0, -5.0) );
  REQUIRE( Approx(7*M_PI_4) == calc_yaw_rad(12.98, -12.98) );

  REQUIRE( Approx(M_PI / 6)      == calc_yaw_rad(sqrt(3), 1.0) );
  REQUIRE( Approx(M_PI / 3)      == calc_yaw_rad(1.0, sqrt(3)) );
  REQUIRE( Approx(2 * M_PI / 3)  == calc_yaw_rad(-1.0, sqrt(3)) );
  REQUIRE( Approx(11 * M_PI / 6) == calc_yaw_rad(sqrt(3), -1.0) );
  REQUIRE( Approx(5 * M_PI / 6)  == calc_yaw_rad(-sqrt(3), 1.0) );
  REQUIRE( Approx(5 * M_PI / 3)  == calc_yaw_rad(1.0, -sqrt(3)) );
  REQUIRE( Approx(7 * M_PI / 6)  == calc_yaw_rad(-sqrt(3), -1.0) );
  REQUIRE( Approx(4 * M_PI / 3)  == calc_yaw_rad(-1.0, -sqrt(3)) );
}

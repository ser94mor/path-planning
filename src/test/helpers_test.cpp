//
// Created by aoool on 8/22/18.
//
#include "catch.hpp"

#include "helpers.hpp"

TEST_CASE("calc_speed", "[helpers]") {
  REQUIRE( calc_speed(200.0, 100.0, 10.0) == -10.0 );
  REQUIRE( calc_speed(100.0, 100.0, 10.0) ==   0.0 );
  REQUIRE( calc_speed(100.0, 200.0, 10.0) ==  10.0 );
}

TEST_CASE("calc_acc", "[helpers]") {
  REQUIRE( calc_acc(800.0, 600.0, 200.0, 10.0) == -2.0 );
  REQUIRE( calc_acc(400.0, 300.0, 100.0, 10.0) == -1.0 );
  REQUIRE( calc_acc(100.0, 200.0, 300.0, 10.0) ==  0.0 );
  REQUIRE( calc_acc(100.0, 200.0, 400.0, 10.0) ==  1.0 );
  REQUIRE( calc_acc(200.0, 400.0, 800.0, 10.0) ==  2.0 );
}

TEST_CASE("calc_jerk", "[helpers]") {
  REQUIRE( calc_jerk(1000.0, 900.0, 700.0, 100.0, 10.0)  == -0.3 );
  REQUIRE( calc_jerk(900.0,  800.0, 600.0, 100.0,  10.0) == -0.2 );
  REQUIRE( calc_jerk(800.0,  700.0, 500.0, 100.0,  10.0) == -0.1 );
  REQUIRE( calc_jerk(100.0,  200.0, 400.0, 700.0,  10.0) ==  0.0 );
  REQUIRE( calc_jerk(100.0,  200.0, 400.0, 800.0,  10.0) ==  0.1 );
  REQUIRE( calc_jerk(100.0,  200.0, 400.0, 900.0,  10.0) ==  0.2 );
  REQUIRE( calc_jerk(100.0,  200.0, 400.0, 1000.0, 10.0) ==  0.3 );
}

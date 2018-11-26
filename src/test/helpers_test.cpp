//
// Created by aoool on 8/22/18.
//
#include "catch.hpp"

#include "helpers.hpp"

TEST_CASE("Calc1DSpeed", "[helpers]") {
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

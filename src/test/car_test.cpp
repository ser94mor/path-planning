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
    .x_m = 1.1,
    .y_m = 2.2,
    .s_m = 3.3,
    .d_m = 4.4,
    .vel_mps = 5.5,
    .vel_x_mps = 6.6,
    .vel_y_mps = 7.7,
    .yaw_rad = 8.8,
    .acc_s_mps2 = 9.9,
    .acc_d_mps2 = 10.1
  };

  std::string expected_res =
      "Car{\n"
      "  .id         = -1,\n"
      "  .state      = PLCR,\n"
      "  .x_m        = 1.100000,\n"
      "  .y_m        = 2.200000,\n"
      "  .s_m        = 3.300000,\n"
      "  .d_m        = 4.400000,\n"
      "  .vel_mps    = 5.500000,\n"
      "  .vel_x_mps  = 6.600000,\n"
      "  .vel_y_mps  = 7.700000,\n"
      "  .yaw_rad    = 8.800000,\n"
      "  .acc_s_mps2 = 9.900000,\n"
      "  .acc_d_mps2 = 10.100000,\n"
      "}";

  oss << car;

  REQUIRE( oss.str() == expected_res );
}

TEST_CASE("Car::FromVector", "[car]") {
  std::vector<double> car_vect{ 12 /*id*/, 37.9 /*x*/, -49.4 /*y*/, -1.9 /*vx*/, 6.5 /*vy*/, 127.8 /*s*/, 0.45 /*d*/};

  Car car = Car::FromVector(car_vect);

  REQUIRE( car.id    == 12 );
  REQUIRE( car.state == State::KeepLane );
  REQUIRE( car.x_m   == Approx(37.9) );
  REQUIRE( car.y_m   == Approx(-49.4) );
  REQUIRE( car.vel_mps == Approx(6.772001181) );
  REQUIRE( car.vel_x_mps == Approx(-1.9) );
  REQUIRE( car.vel_y_mps == Approx(6.5) );
  REQUIRE( car.yaw_rad == Approx(1.8551811022) );
  REQUIRE( car.acc_s_mps2 == Approx(0.0) );
  REQUIRE( car.acc_d_mps2 == Approx(0.0) );
}

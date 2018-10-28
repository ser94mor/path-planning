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
  };

  std::string expected_res =
      "Car{\n"
      "  .id        = -1,\n"
      "  .state     = PLCR,\n"
      "  .x_m       = 1.100000,\n"
      "  .y_m       = 2.200000,\n"
      "  .s_m       = 3.300000,\n"
      "  .d_m       = 4.400000,\n"
      "  .vel_mps   = 5.500000,\n"
      "  .vel_x_mps = 6.600000,\n"
      "  .vel_y_mps = 7.700000,\n"
      "  .yaw_rad   = 8.800000,\n"
      "}";

  oss << car;

  REQUIRE( oss.str() == expected_res );
}

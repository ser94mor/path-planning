//
// Created by aoool on 28.10.18.
//

#include "state.hpp"

#include <catch.hpp>
#include <ostream>

TEST_CASE("operator<<(State)", "[state]") {
  std::ostringstream oss;

  oss.str(std::string{});
  oss << State::KeepLane;
  REQUIRE( oss.str() == "KL" );

  oss.str(std::string{});
  oss << State::PrepareLaneChangeLeft;
  REQUIRE( oss.str() == "PLCL" );

  oss.str(std::string{});
  oss << State::PrepareLaneChangeRight;
  REQUIRE( oss.str() == "PLCR" );

  oss.str(std::string{});
  oss << State::LaneChangeLeft;
  REQUIRE( oss.str() == "LCL" );

  oss.str(std::string{});
  oss << State::LaneChangeRight;
  REQUIRE( oss.str() == "LCR" );
}

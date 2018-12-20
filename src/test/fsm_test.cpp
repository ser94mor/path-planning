//
// Created by aoool on 28.10.18.
//

#include "fsm.hpp"

#include <catch.hpp>
#include <ostream>

TEST_CASE("operator<<(State)", "[fsm]") {
  std::ostringstream oss;

  oss.str(std::string{});
  oss << FSM::State::KeepLane;
  REQUIRE( oss.str() == "KL" );

  oss.str(std::string{});
  oss << FSM::State::PrepareLaneChangeLeft;
  REQUIRE( oss.str() == "PLCL" );

  oss.str(std::string{});
  oss << FSM::State::PrepareLaneChangeRight;
  REQUIRE( oss.str() == "PLCR" );

  oss.str(std::string{});
  oss << FSM::State::LaneChangeLeft;
  REQUIRE( oss.str() == "LCL" );

  oss.str(std::string{});
  oss << FSM::State::LaneChangeRight;
  REQUIRE( oss.str() == "LCR" );
}

TEST_CASE("FSM::GetPossibleNextStates(State)", "[fsm]") {
  std::vector<FSM::State>   kl{ FSM::State::KeepLane,        FSM::State::PrepareLaneChangeLeft,  FSM::State::PrepareLaneChangeRight, };
  std::vector<FSM::State> plcl{ FSM::State::LaneChangeLeft,  FSM::State::PrepareLaneChangeLeft,  FSM::State::KeepLane,               };
  std::vector<FSM::State> plcr{ FSM::State::LaneChangeRight, FSM::State::PrepareLaneChangeRight, FSM::State::KeepLane,               };
  std::vector<FSM::State>  lcl{ FSM::State::KeepLane,        FSM::State::LaneChangeLeft,                                             };
  std::vector<FSM::State>  lcr{ FSM::State::KeepLane,        FSM::State::LaneChangeRight,                                            };

  SECTION("FSM::GetPossibleNextStates produces expected results") {
    REQUIRE(FSM::GetPossibleNextStates(FSM::State::KeepLane) == kl);
    REQUIRE(FSM::GetPossibleNextStates(FSM::State::PrepareLaneChangeLeft) == plcl);
    REQUIRE(FSM::GetPossibleNextStates(FSM::State::PrepareLaneChangeRight) == plcr);
    REQUIRE(FSM::GetPossibleNextStates(FSM::State::LaneChangeLeft) == lcl);
    REQUIRE(FSM::GetPossibleNextStates(FSM::State::LaneChangeRight) == lcr);
  }

  SECTION("FSM::GetPossibleNextStates returns the same reference each time") {
    auto& one = FSM::GetPossibleNextStates(FSM::State::KeepLane);
    auto& two = FSM::GetPossibleNextStates(FSM::State::KeepLane);

    REQUIRE( &one == &two );
  }

}

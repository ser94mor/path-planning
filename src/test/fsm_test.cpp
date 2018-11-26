//
// Created by aoool on 28.10.18.
//

#include "fsm.hpp"

#include <catch.hpp>
#include <ostream>

TEST_CASE("operator<<(State)", "[fsm]") {
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

TEST_CASE("FSM::GetPossibleNextStates(State)", "[fsm]") {
  std::vector<State>   kl{ State::KeepLane, State::PrepareLaneChangeLeft, State::PrepareLaneChangeRight };
  std::vector<State> plcl{ State::PrepareLaneChangeLeft,  State::LaneChangeLeft, State::KeepLane        };
  std::vector<State> plcr{ State::PrepareLaneChangeRight, State::LaneChangeRight, State::KeepLane       };
  std::vector<State>  lcl{ State::LaneChangeLeft,  State::KeepLane                                      };
  std::vector<State>  lcr{ State::LaneChangeRight, State::KeepLane                                      };

  SECTION("FSM::GetPossibleNextStates produces expected results") {
    REQUIRE(FSM::GetPossibleNextStates(State::KeepLane) == kl);
    REQUIRE(FSM::GetPossibleNextStates(State::PrepareLaneChangeLeft) == plcl);
    REQUIRE(FSM::GetPossibleNextStates(State::PrepareLaneChangeRight) == plcr);
    REQUIRE(FSM::GetPossibleNextStates(State::LaneChangeLeft) == lcl);
    REQUIRE(FSM::GetPossibleNextStates(State::LaneChangeRight) == lcr);
  }

  SECTION("FSM::GetPossibleNextStates returns the same reference each time") {
    auto& one = FSM::GetPossibleNextStates(State::KeepLane);
    auto& two = FSM::GetPossibleNextStates(State::KeepLane);

    REQUIRE( &one == &two );
  }

}

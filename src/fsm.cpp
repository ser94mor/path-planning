//
// Created by aoool on 17.11.18.
//

#include "fsm.hpp"

std::unordered_map< State, const std::vector<State> >  FSM::transition_map_{
    { State::KeepLane,               { State::KeepLane, State::PrepareLaneChangeLeft, State::PrepareLaneChangeRight } },
    { State::PrepareLaneChangeLeft,  { State::PrepareLaneChangeLeft,  State::LaneChangeLeft,  State::KeepLane       } },
    { State::PrepareLaneChangeRight, { State::PrepareLaneChangeRight, State::LaneChangeRight, State::KeepLane       } },
    { State::LaneChangeLeft,         { State::LaneChangeLeft,  State::KeepLane                                      } },
    { State::LaneChangeRight,        { State::LaneChangeRight, State::KeepLane                                      } },
  };

const std::vector<State>& FSM::GetPossibleNextStates(State state) {
  return transition_map_[state];
}

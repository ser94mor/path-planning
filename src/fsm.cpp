//
// Created by aoool on 17.11.18.
//

#include "fsm.hpp"

std::unordered_map< FSM::State, const std::vector<FSM::State> >  FSM::transition_map_{
    { State::KeepLane,               { State::KeepLane, State::PrepareLaneChangeLeft, State::PrepareLaneChangeRight } },
    { State::PrepareLaneChangeLeft,  { State::LaneChangeLeft,  State::PrepareLaneChangeLeft,  State::KeepLane       } },
    { State::PrepareLaneChangeRight, { State::LaneChangeRight, State::PrepareLaneChangeRight, State::KeepLane       } },
    { State::LaneChangeLeft,         { State::KeepLane, State::LaneChangeLeft,                                      } },
    { State::LaneChangeRight,        { State::KeepLane, State::LaneChangeRight,                                     } },
  };

const std::vector<FSM::State>& FSM::GetPossibleNextStates(State state) {
  return transition_map_[state];
}

//
// Created by aoool on 17.11.18.
//

#include "fsm.hpp"

std::unordered_map< FSM::State, const std::vector<FSM::State> >  FSM::transition_map_{
  { State::KeepLane,               { State::KeepLane, State::LaneChangeLeft, State::LaneChangeRight } },
  { State::LaneChangeLeft,         { State::KeepLane, State::LaneChangeLeft, } },
  { State::LaneChangeRight,        { State::KeepLane, State::LaneChangeRight, } },
};

std::unordered_set<FSM::State> FSM::lane_changing_states_{
  State::LaneChangeLeft, State::LaneChangeRight,
};

std::unordered_set<FSM::State> FSM::lane_keeping_states_{
  State::KeepLane, State::PrepareLaneChangeLeft, State::PrepareLaneChangeRight,
};

const std::vector<FSM::State>& FSM::GetPossibleNextStates(State state) {
  return transition_map_[state];
}

const std::unordered_set<FSM::State>& FSM::GetLaneChangingStates() {
  return lane_changing_states_;
}

const std::unordered_set<FSM::State>& FSM::GetLaneKeepingStates() {
  return lane_keeping_states_;
}

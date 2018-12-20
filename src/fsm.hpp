//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_STATE_HPP
#define PATH_PLANNING_STATE_HPP

#include <iostream>
#include <typeinfo>
#include <sstream>
#include <vector>
#include <unordered_map>

class FSM {
public:
  enum class State {
    KeepLane,
    PrepareLaneChangeLeft,
    PrepareLaneChangeRight,
    LaneChangeLeft,
    LaneChangeRight,
  };

  static const std::vector<State>& GetPossibleNextStates(State state);

private:
  static std::unordered_map< State, const std::vector<State> >  transition_map_;
};

inline std::ostream& operator <<(std::ostream& ostream, const FSM::State state) {
  switch (state) {
    case FSM::State::KeepLane:
      ostream << "KL";
      break;
    case FSM::State::PrepareLaneChangeLeft:
      ostream << "PLCL";
      break;
    case FSM::State::PrepareLaneChangeRight:
      ostream << "PLCR";
      break;
    case FSM::State::LaneChangeLeft:
      ostream << "LCL";
      break;
    case FSM::State::LaneChangeRight:
      ostream << "LCR";
      break;
    default:
      std::cerr  << __PRETTY_FUNCTION__ <<  " must handle " << state << " state" << std::endl;
      std::exit(EXIT_FAILURE);
  }

  return ostream;
}


#endif //PATH_PLANNING_STATE_HPP

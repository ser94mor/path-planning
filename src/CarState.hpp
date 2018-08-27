//
// Created by aoool on 8/22/18.
//

#ifndef PATH_PLANNING_CARSTATE_HPP
#define PATH_PLANNING_CARSTATE_HPP


struct CarState {
  double x;
  double y;
  double s;
  double d;
  double speed_x;
  double speed_y;
  double speed_s;
  double speed_d;
  double acc_x;
  double acc_y;
  double acc_s;
  double acc_d;
};

#endif //PATH_PLANNING_CARSTATE_HPP

//
// Created by aoool on 8/22/18.
//

#ifndef PATH_PLANNING_HELPERS_HPP
#define PATH_PLANNING_HELPERS_HPP

#include <cmath>

inline double calc_speed(double p1, double p2, double time) {
  return (p2 - p1) / time;
}

inline double calc_acc(double p1, double p2, double p3, double time) {
  return (calc_speed(p2, p3, time) - calc_speed(p1, p2, time)) / time;
}

inline double calc_jerk(double p1, double p2, double p3, double p4, double time) {
  return (calc_acc(p2, p3, p4, time) - calc_acc(p1, p2, p3, time)) / time;
}

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }

inline double deg2rad(double x) { return x * pi() / 180; }

inline double rad2deg(double x) { return x * 180 / pi(); }


inline double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

#endif //PATH_PLANNING_HELPERS_HPP

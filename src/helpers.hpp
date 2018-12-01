//
// Created by aoool on 8/22/18.
//

#ifndef PATH_PLANNING_HELPERS_HPP
#define PATH_PLANNING_HELPERS_HPP

#include "path_planner_config.hpp"

#include <cmath>
#include <cstdint>
#include <vector>


inline double Calc1DSpeed(double p1, double p2, double time)
{
  return (p2 - p1) / time;
}


inline double Calc1DAcc(double p1, double p2, double p3, double time) {
  return (Calc1DSpeed(p2, p3, time) - Calc1DSpeed(p1, p2, time)) / time;
}


inline double Calc1DJerk(double p1, double p2, double p3, double p4, double time)
{
  return (Calc1DAcc(p2, p3, p4, time) - Calc1DAcc(p1, p2, p3, time)) / time;
}


inline double Calc2DVectorLen(double x1, double x2)
{
  return sqrt(x1*x1 + x2*x2);
}


inline double CalcAbsVelocity(double vx, double vy)
{
  return Calc2DVectorLen(vx, vy);
}


// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }


inline double DegToRad(double x) { return x * pi() / 180; }


inline double RadToDeg(double x) { return x * 180 / pi(); }


inline double CalcDistance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Convert value in miles per hour unit to meters per second.
inline constexpr double MphToMps(double mph)
{
  return mph * 0.44704;
}

// Calculate yaw rate based on the v_x and v_y velocities
inline double CalcYawRad(double vx, double vy)
{
  double v = sqrt(vx*vx + vy*vy);
  double asin_angle = asin(vy / v);

  if (vx >=0 && vy >= 0) {
    return asin_angle;            // asin is positive here
  } else if (vx >= 0 && vy < 0) {
    return 2 * pi() + asin_angle; // asin is negative here
  } else {
    return pi() - asin_angle;
  }
}

inline double Calc1DPosition(double x, double v, double a, double t)
{
  return x + v*t + ((a*t)/2)*t;
}

int ClosestWaypoint(double x, double y, const PathPlannerConfig& config);

int NextWaypoint(double x, double y, double vx, double vy, const PathPlannerConfig& config);

std::vector<double> GetFrenet(double x, double y, double vx, double vy, const PathPlannerConfig& config);

std::vector<double> GetXY(double s, double d, const PathPlannerConfig& config);

std::vector<double> GetVxVy(double s, double d, double vs, double vd, const PathPlannerConfig& config);

std::vector<double> GetFrenetSpeed(double s, double d, double x, double y, double vel_x, double vel_y,
                                   const PathPlannerConfig& config);


#endif //PATH_PLANNING_HELPERS_HPP

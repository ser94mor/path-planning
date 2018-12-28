//
// Created by aoool on 8/22/18.
//

#ifndef PATH_PLANNING_HELPERS_HPP
#define PATH_PLANNING_HELPERS_HPP

#include "path_planner_config.hpp"

#include <cmath>
#include <cstdint>
#include <vector>
#include <map>
#include <numeric>
#include <algorithm>
#include <cassert>


template<typename T, typename U> std::vector<T> map_keys(const std::map<T, U>& map) {
  std::vector<T> keys(map.size());
  int ind = 0;
  for (const auto& entry : map) {
    keys[ind++] = entry.first;
  }

  return keys;
}

template<typename T, typename U> std::vector<U> map_vals(const std::map<T, U>& map) {
  std::vector<U> vals(map.size());
  int ind = 0;
  for (const auto& entry : map) {
    vals[ind++] = entry.second;
  }

  return vals;
}

inline double Calc1DVelocity(double p1, double p2, double time)
{
  return (p2 - p1) / time;
}


inline double Calc1DVelocity(double vel, double acc, double jerk, double t)
{
  return vel + acc*t + ((jerk*t)/2)*t;
}


inline double Calc1DAcc(double p1, double p2, double p3, double time) {
  return (Calc1DVelocity(p2, p3, time) - Calc1DVelocity(p1, p2, time)) / time;
}


inline double Calc1DAcc(double v1, double v2, double t)
{
  // acceleration is a speed of velocity change within the time frame t
  return Calc1DVelocity(v1, v2, t);
}


inline double Calc1DJerk(double p1, double p2, double p3, double p4, double time)
{
  return (Calc1DAcc(p2, p3, p4, time) - Calc1DAcc(p1, p2, p3, time)) / time;
}


// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }


inline double DegToRad(double x) { return x * pi() / 180; }


inline double RadToDeg(double x) { return x * 180 / pi(); }


inline double EuclideanNorm(const std::vector<double>& v)
{
  return std::sqrt(std::inner_product(v.begin(), v.end(), v.begin(), 0.0));
}


inline double EuclideanMetric(const std::vector<double>& v1, const std::vector<double>& v2)
{
  assert(v1.size() == v2.size());
  std::vector<double> diff(v1.size());
  std::transform(v1.begin(), v1.end(), v2.begin(), diff.begin(), std::minus<>());
  return std::sqrt(std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0));
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


inline double CalcPolynomial(const std::vector<double>& coeffs, double x)
{
  double res = 0.0;
  double arg = 1.0;
  for (auto coeff : coeffs) {
    res += coeff * arg;
    arg *= x;
  }

  return res;
}


inline double CalcPolynomialDerivative(const std::vector<double>& coeffs, double x, double order)
{
  if (order == 0) {
    return CalcPolynomial(coeffs, x);
  } else {
    if (coeffs.size() <= 1) {
      return 0.0;
    } else {
      std::vector<double> new_coeffs(coeffs.size() - 1);
      std::iota(new_coeffs.begin(), new_coeffs.end(), 1.0);
      std::transform(coeffs.begin() + 1, coeffs.end(), new_coeffs.begin(), new_coeffs.begin(), std::multiplies<>());
      return CalcPolynomialDerivative(new_coeffs, x , --order);
    }
  }
}


inline bool IsEqual(double d1, double d2) 
{
  return (fabs(d1 - d2) < 0.000000001);
}


inline double Logistic(double x) {
  return 2.0 / (1 + exp(-x)) - 1.0;
}


int ClosestWaypoint(double x, double y, const PathPlannerConfig& config);

int NextWaypoint(double x, double y, double vx, double vy, const PathPlannerConfig& config);

std::vector<double> GetFrenet(double x, double y, double vx, double vy, const PathPlannerConfig& config);

std::vector<double> GetXY(double s, double d, const PathPlannerConfig& config);

std::vector<double> GetVxVy(double s, double d, double vs, double vd, const PathPlannerConfig& config);


#endif //PATH_PLANNING_HELPERS_HPP

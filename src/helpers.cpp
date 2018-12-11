//
// Created by aoool on 16.11.18.
//

#include "helpers.hpp"
#include "path_planner_config.hpp"

#include <limits>
#include <vector>
#include <iostream>

int ClosestWaypoint(double x, double y, const PathPlannerConfig& config) {

  double closest_len = std::numeric_limits<double>::max();
  int closest_waypoint = 0;

  for (auto i = 0; i < config.map_wps_x_m.size(); i++) {
    double map_x = config.map_wps_x_m[i];
    double map_y = config.map_wps_y_m[i];
    double dist = CalcDistance(x, y, map_x, map_y);
    if (dist < closest_len) {
      closest_len = dist;
      closest_waypoint = i;
    }

  }

  return closest_waypoint;
}

int NextWaypoint(double x, double y, double vx, double vy, const PathPlannerConfig& config) {

  int closest_waypoint = ClosestWaypoint(x, y, config);

  double map_x = config.map_wps_x_m[closest_waypoint];
  double map_y = config.map_wps_y_m[closest_waypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double theta = CalcYawRad(vx, vy);

  double angle = fabs(theta - heading);
  angle = fmin(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closest_waypoint++;
    if (closest_waypoint == config.map_wps_x_m.size()) {
      closest_waypoint = 0;
    }
  }

  return closest_waypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> GetFrenet(double x, double y, double vx, double vy, const PathPlannerConfig& config) {
  int next_wp = NextWaypoint(x, y, vx, vy, config);

  int prev_wp = next_wp == 0 ? static_cast<int>(config.map_wps_x_m.size() - 1) : next_wp - 1;

  double n_x = config.map_wps_x_m[next_wp] - config.map_wps_x_m[prev_wp];
  double n_y = config.map_wps_y_m[next_wp] - config.map_wps_y_m[prev_wp];
  double x_x = x - config.map_wps_x_m[prev_wp];
  double x_y = y - config.map_wps_y_m[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = CalcDistance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - config.map_wps_x_m[prev_wp];
  double center_y = 2000 - config.map_wps_y_m[prev_wp];
  double center_to_pos = CalcDistance(center_x, center_y, x_x, x_y);
  double center_to_ref = CalcDistance(center_x, center_y, proj_x, proj_y);

  if (center_to_pos <= center_to_ref) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += CalcDistance(config.map_wps_x_m[i],
                             config.map_wps_y_m[i],
                             config.map_wps_x_m[i + 1],
                             config.map_wps_y_m[i + 1]);
  }

  frenet_s += CalcDistance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

std::vector<double> GetXY(double s, double d, const PathPlannerConfig& config)
{
  double x = config.spline_s_x(s) + d * config.spline_s_dx(s);
  double y = config.spline_s_y(s) + d * config.spline_s_dy(s);

  return { x, y };
}

std::vector<double> GetVxVy(double s, double d, double vs, double vd, const PathPlannerConfig& config)
{
  const double t = config.frequency_s;

  auto cur_coords = GetXY(s,d, config);
  auto future_coords = GetXY(s + vs * t, d + vd * t, config);

  double vx = Calc1DVelocity(cur_coords[0], future_coords[0], t);
  double vy = Calc1DVelocity(cur_coords[1], future_coords[1], t);

  return { vx, vy };
}

std::vector<double> GetFrenetSpeed(double s, double d, double x, double y, double vel_x, double vel_y,
                                   const PathPlannerConfig& config)
{
  // since we need to calculate s_dot and d_dot, we simply assume that the object is moving with the constant velocity
  double next_x = Calc1DPosition(x, vel_x, 0, config.frequency_s);
  double next_y = Calc1DPosition(y, vel_y, 0, config.frequency_s);

  double yaw = CalcYawRad(vel_x, vel_y);

  auto frenet_coords = GetFrenet(next_x, next_y, vel_x, vel_y, config);
  double next_s = frenet_coords[0];
  double next_d = frenet_coords[1];

  double s_dot = next_s - s;
  double d_dot = next_d - d;

  return { s_dot, d_dot };
}

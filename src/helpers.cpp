//
// Created by aoool on 16.11.18.
//

#include "helpers.hpp"
#include "path_planner_config.hpp"

#include <limits>
#include <vector>
#include <iostream>
#include <spline.h>

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y) {

  double closest_len = std::numeric_limits<double>::max();
  int closest_waypoint = 0;

  for (auto i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = CalcDistance(x, y, map_x, map_y);
    if (dist < closest_len) {
      closest_len = dist;
      closest_waypoint = i;
    }

  }

  return closest_waypoint;
}

int NextWaypoint(double x, double y, double theta,
                 const std::vector<double> &maps_x, const std::vector<double> &maps_y) {

  int closest_waypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closest_waypoint];
  double map_y = maps_y[closest_waypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = fmin(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closest_waypoint++;
    if (closest_waypoint == maps_x.size()) {
      closest_waypoint = 0;
    }
  }

  return closest_waypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> GetFrenet(double x, double y, double theta, const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp = next_wp == 0 ? static_cast<int>(maps_x.size() - 1) : next_wp - 1;

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = CalcDistance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double center_to_pos = CalcDistance(center_x, center_y, x_x, x_y);
  double center_to_ref = CalcDistance(center_x, center_y, proj_x, proj_y);

  if (center_to_pos <= center_to_ref) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += CalcDistance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += CalcDistance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double>
GetXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x,
      const std::vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < maps_s.size() - 1)) {
    prev_wp++;
  }

  auto wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}


std::vector<double>
GetXY(double s, double d, const PathPlannerConfig& config) {
  auto size = config.map_wps_s_m.size();

  // using pointer and modular arithmetic to calculate previous waypoint for s
  auto iter = std::lower_bound(config.map_wps_s_m.begin(),
                               config.map_wps_s_m.end(),
                               s);
  auto prev_wp = (size + std::distance(config.map_wps_s_m.begin(), iter) - 1) % size;

  // for spline, take previous waypoint and 2 more from both sides
  std::vector<uint64_t> inds{
    (prev_wp - 3 + size) % size,
    (prev_wp - 2 + size) % size,
    (prev_wp - 1 + size) % size,
    prev_wp,
    (prev_wp + 1) % size,
    (prev_wp + 2) % size,
    (prev_wp + 3) % size,
  };

  // track is circular, s[i] may be greater than s[i+1], so we need to make some s-es negative
  std::vector<double> s_pts(inds.size());
  bool switched = false;
  for (int i = static_cast<int>(inds.size() - 1); i >= 0; --i) {
    s_pts[i] = switched ? (config.max_s_m - config.map_wps_s_m[i]) : config.map_wps_s_m[i];
    switched = false; //(i > 0) && (config.map_wps_s_m[i-1] > config.map_wps_s_m[i]);
  }

  std::vector<double> map_wps_x(inds.size());
  std::vector<double> map_wps_y(inds.size());
  std::vector<double> map_wps_dx(inds.size());
  std::vector<double> map_wps_dy(inds.size());
  for (int i = 0; i < inds.size(); ++i) {
    map_wps_x[i]  = config.map_wps_x_m[inds[i]];
    map_wps_y[i]  = config.map_wps_y_m[inds[i]];
    map_wps_dx[i] = config.map_wps_dx_m[inds[i]];
    map_wps_dy[i] = config.map_wps_dy_m[inds[i]];
  }

  tk::spline spline_s_to_x;
  spline_s_to_x.set_points(s_pts, map_wps_x);

  tk::spline spline_s_to_y;
  spline_s_to_y.set_points(s_pts, map_wps_y);

  tk::spline spline_s_to_dx;
  spline_s_to_dx.set_points(s_pts, map_wps_dx);

  tk::spline spline_s_to_dy;
  spline_s_to_dy.set_points(s_pts, map_wps_dy);

  double x = spline_s_to_x(s) + d * spline_s_to_dx(s);
  double y = spline_s_to_y(s) + d * spline_s_to_dy(s);

  return { x, y };
}

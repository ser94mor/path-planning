//
// Created by aoool on 8/8/18.
//

#include "PathPlanner.hpp"
#include "Car.hpp"
#include "helpers.hpp"

#include <cmath>
#include <cassert>
#include <limits>
#include <iostream>
#include <iomanip>


ulong ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y) {

  double closestLen = std::numeric_limits<double>::max();
  ulong closestWaypoint = 0;

  for (ulong i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;
}

ulong NextWaypoint(double x, double y, double theta,
                   const std::vector<double> &maps_x, const std::vector<double> &maps_y) {

  ulong closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = fmin(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> GetFrenet(double x, double y, double theta, const std::vector<double> &maps_x,
                              const std::vector<double> &maps_y) {
  auto next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  auto prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double>
getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
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

std::vector<double> PathPlanner::GetPrevXY(Car &car,
                                           std::vector<double> &prev_path_x,
                                           std::vector<double> &prev_path_y,
                                           int back_offset) {
  if (not invoked_) {
    // the first invocation of the path planner happens when car does not move, thus all previous coords are the same
    return { car.x_m, car.y_m };
  }

  size_t prev_size = prev_path_x.size();
  if ( prev_size > back_offset ) {

    // coordinates are stored in prev_path_x and prev_path_y vectors
    size_t index = prev_size - back_offset - 1;
    return { prev_path_x[index], prev_path_y[index] };

  } else {

    // coordinates are stored in next_coords_ vector
    size_t index = path_len_ + prev_size - back_offset - 1;
    return { next_coords_[0][index], next_coords_[1][index] };

  }
}

CarState PathPlanner::GetCarState(Car &car,
                                  std::vector<double> &prev_path_x,
                                  std::vector<double> &prev_path_y) {
  auto xy_coords_t4 = GetPrevXY(car, prev_path_x, prev_path_y, 0);
  auto xy_coords_t3 = GetPrevXY(car, prev_path_x, prev_path_y, 1);
  auto xy_coords_t2 = GetPrevXY(car, prev_path_x, prev_path_y, 2);
  auto xy_coords_t1 = GetPrevXY(car, prev_path_x, prev_path_y, 3);

  double yaw_rad_t4 = atan2(xy_coords_t4[1] - xy_coords_t3[1], xy_coords_t4[0] - xy_coords_t3[0]);
  double yaw_rad_t3 = atan2(xy_coords_t3[1] - xy_coords_t2[1], xy_coords_t3[0] - xy_coords_t2[0]);
  double yaw_rad_t2 = atan2(xy_coords_t2[1] - xy_coords_t1[1], xy_coords_t2[0] - xy_coords_t1[0]);

  auto sd_coords_t4 = GetFrenet(xy_coords_t4[0], xy_coords_t4[1], yaw_rad_t4, map_waypoints_x_m_, map_waypoints_y_m_);
  auto sd_coords_t3 = GetFrenet(xy_coords_t3[0], xy_coords_t3[1], yaw_rad_t3, map_waypoints_x_m_, map_waypoints_y_m_);
  auto sd_coords_t2 = GetFrenet(xy_coords_t2[0], xy_coords_t2[1], yaw_rad_t2, map_waypoints_x_m_, map_waypoints_y_m_);

  double speed_x = calc_speed(xy_coords_t3[0], xy_coords_t4[0], frequency_s_);
  double speed_y = calc_speed(xy_coords_t3[1], xy_coords_t4[1], frequency_s_);
  double speed_s = calc_speed(sd_coords_t3[0], sd_coords_t4[0], frequency_s_);
  double speed_d = calc_speed(sd_coords_t3[1], sd_coords_t4[1], frequency_s_);

  double acc_x = calc_acc(xy_coords_t2[0], xy_coords_t3[0], xy_coords_t4[0], frequency_s_);
  double acc_y = calc_acc(xy_coords_t2[1], xy_coords_t3[1], xy_coords_t4[1], frequency_s_);
  double acc_s = calc_acc(sd_coords_t2[0], sd_coords_t3[0], sd_coords_t4[0], frequency_s_);
  double acc_d = calc_acc(sd_coords_t2[1], sd_coords_t3[1], sd_coords_t4[1], frequency_s_);

  CarState car_state = {
      .x = xy_coords_t4[0],
      .y = xy_coords_t4[1],
      .s = sd_coords_t4[0],
      .d = sd_coords_t4[1],
      .speed_x = speed_x,
      .speed_y = speed_y,
      .speed_s = speed_s,
      .speed_d = speed_d,
      .acc_x = acc_x,
      .acc_y = acc_y,
      .acc_s = acc_s,
      .acc_d = acc_d,
  };

  return car_state;
}


static int counter = 0;

std::vector<std::vector<double> > &PathPlanner::GetNextXYTrajectories(Car &car,
                                                                      std::vector<double> &prev_path_x,
                                                                      std::vector<double> &prev_path_y,
                                                                      double end_path_s,
                                                                      double end_path_d,
                                                                      std::vector< std::vector<double> > &sensor_fusion)
{
  assert( prev_path_x.size() == prev_path_y.size() );
  auto prev_path_size = prev_path_x.size();

  std::cout << "======" << std::endl;
  std::cout << car.d_m << std::endl;

  for (int i = 0; i < prev_path_size; i++) {
    // we do not re-plan the route
    next_coords_[0][i] = prev_path_x[i];
    next_coords_[1][i] = prev_path_y[i];
  }

  double s = prev_s_m_;
  double d = prev_d_m_;
  double speed_s = prev_speed_mps_;
  double acc_s   = prev_acc_mps2_;
  double jerk_s  = max_jerk_mps3_;

  for (auto i = prev_path_size; i < path_len_; i++) {
    if (speed_s < target_speed_mps_) {
      acc_s += max_jerk_mps3_ * frequency_s_;
      if (acc_s > max_acc_mps2_) {
        acc_s = max_acc_mps2_;
      }
    } else if (speed_s > target_speed_mps_) {
      acc_s -= jerk_s * frequency_s_;
      if (-acc_s > max_acc_mps2_) {
        acc_s = -max_acc_mps2_;
      }
    }

    speed_s += acc_s * frequency_s_;
    //speed_pid_ctrl_.UpdateError()

    s += speed_s * frequency_s_;

    std::vector<double> coords = getXY(s, d, map_waypoints_s_m_, map_waypoints_x_m_, map_waypoints_y_m_);

    next_coords_[0][i] = coords[0];
    next_coords_[1][i] = coords[1];

    prev_acc_mps2_ = acc_s;
    prev_speed_mps_ = speed_s;
    prev_s_m_ = s;
    prev_d_m_ = d;

    std::cout << ++counter << " s: " << s << "; speed_s: " << speed_s << "; acc_s: " << acc_s << std::endl;
  }

  invoked_ = true;

  return next_coords_;
}


PathPlanner::~PathPlanner() = default;

PathPlanner::PathPlanner(double frequency_s,
                         double target_speed_mps,
                         double max_acceleration_mps2,
                         double max_jerk_mps3,
                         size_t path_len,
                         size_t lane_num,
                         std::vector<double> const &map_waypoints_x_m,
                         std::vector<double> const &map_waypoints_y_m,
                         std::vector<double> const &map_waypoints_s_m,
                         std::vector<double> const &map_waypoints_dx_m,
                         std::vector<double> const &map_waypoints_dy_m):
                         frequency_s_(frequency_s),
                         target_speed_mps_(target_speed_mps),
                         max_acc_mps2_(max_acceleration_mps2),
                         max_jerk_mps3_(max_jerk_mps3),
                         path_len_(path_len),
                         lane_num_(lane_num),
                         next_coords_(2, std::vector<double>(path_len_)),
                         map_waypoints_x_m_(map_waypoints_x_m),
                         map_waypoints_y_m_(map_waypoints_y_m),
                         map_waypoints_s_m_(map_waypoints_s_m),
                         map_waypoints_d_x_m_(map_waypoints_dx_m),
                         map_waypoints_d_y_m_(map_waypoints_dy_m),
                         invoked_(false),
                         prev_acc_mps2_(0.0),
                         prev_s_m_(0.0),
                         prev_d_m_(6.16483),
                         prev_speed_mps_(0.0),
                         speed_pid_ctrl_()
{
  speed_pid_ctrl_.Init(0.2, 0.0, 3.0, 0, 0, 0.0, 0.0, 0.0, 0.0);

  std::cout << "|||||||||||||||||||||||||||||||||||||||||||||\n"
            << "||               PATH PLANNER              ||\n"
            << "|| frequency            : " << std::setw(10) << frequency_s_      << " s     ||\n"
            << "|| target speed         : " << std::setw(10) << target_speed_mps_ << " m/s   ||\n"
            << "|| maximum acceleration : " << std::setw(10) << max_acc_mps2_     << " m/s^2 ||\n"
            << "|| maximum jerk         : " << std::setw(10) << max_jerk_mps3_    << " m/s^3 ||\n"
            << "|| path length          : " << std::setw(10) << path_len_         << " -     ||\n"
            << "|| number of lanes      : " << std::setw(10) << lane_num_         << " -     ||\n"
            << "|| start speed          : " << std::setw(10) << prev_speed_mps_   << " m/s   ||\n"
            << "|| start acceleration   : " << std::setw(10) << prev_acc_mps2_    << " m/s^2 ||\n"
            << "|||||||||||||||||||||||||||||||||||||||||||||\n"
            << std::endl;
}

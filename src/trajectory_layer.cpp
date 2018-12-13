//
// Created by aoool on 15.11.18.
//

#include "trajectory_layer.hpp"

#include <typeinfo>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;


std::vector<double>
TrajectoryLayer::GetJerkMinimizingTrajectory(std::vector<double> start, std::vector<double> end, double t) const
{
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time t.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  t     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */

  MatrixXd A = MatrixXd(3, 3);
  A << t*t*t, t*t*t*t, t*t*t*t*t,
       3*t*t, 4*t*t*t, 5*t*t*t*t,
         6*t,  12*t*t,  20*t*t*t;

  MatrixXd B = MatrixXd(3,1);
  B << end[0]-(start[0]+start[1]*t+.5*start[2]*t*t),
       end[1]-(start[1]+start[2]*t),
       end[2]-start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai*B;

  std::vector<double> result = {start[0], start[1], 0.5*start[2]};
  for(int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }

  return result;

}


TrajectoryLayer::TrajectoryLayer(const PathPlannerConfig& config, LocalizationLayer& localization_layer,
                                 PredictionLayer& prediction_layer, BehaviorLayer& behavior_layer):
  pp_config_{config},
  localization_layer_{localization_layer},
  prediction_layer_{prediction_layer},
  behavior_layer_{behavior_layer},
  initialized_{false},
  ego_car_{},
  next_cars_{}
{

}


std::vector<Car>
TrajectoryLayer::GetTrajectory(size_t num_points)
{
  if (!initialized_) {
    throw std::logic_error("TrajectoryLayer::Initialize should be invoked before TrajectoryLayer::GetTrajectory");
  }

  auto&& predictions = prediction_layer_.GetPredictions(num_points * pp_config_.frequency_s, ego_car_.time_s);

  auto&& cur_other_cars = map_keys(predictions);
  auto&& cur_other_cars_same_lane = GetCarsInLane(ego_car_.Lane(), pp_config_.lane_width_m, cur_other_cars);
  std::optional<Car> cur_other_car_ahead_opt =
      GetNearestCarAheadBySCoordIfPresent(ego_car_, cur_other_cars_same_lane);

  if (cur_other_car_ahead_opt.has_value()) {
    auto cur_other_car_ahead = cur_other_car_ahead_opt.value();

    if (ego_car_.IsFrontBufferViolatedBy(cur_other_car_ahead)) {
      std::cout << typeid(this).name() << "::GetTrajectory identified the front buffer violation of ego car\n"
                << ego_car_ << "\n by other car\n" << cur_other_car_ahead << std::endl;
      next_cars_.resize(0);
    }
  }


  if (next_cars_.size() >= pp_config_.path_len) {
    std::vector<Car> to_return{next_cars_.rbegin(), next_cars_.rbegin() + num_points};
    ego_car_ = to_return[to_return.size() - 1];
    next_cars_.resize(next_cars_.size() - num_points);

    return to_return;
  }

  if (!next_cars_.empty()) {
    ego_car_ = next_cars_[0];
  }

  Car planned_ego_car = behavior_layer_.Plan(ego_car_);

  double ego_car_s = static_cast<double>(ego_car_.s_m);
  double planned_ego_car_s = static_cast<double>(planned_ego_car.s_m);
  if (planned_ego_car_s < ego_car_s) {
    planned_ego_car_s += pp_config_.max_s_m;
  }

  double planning_time_horizon = planned_ego_car.time_s - ego_car_.time_s;

  std::vector<double> s_coeffs = GetJerkMinimizingTrajectory(
      {ego_car_s, ego_car_.vel_s_mps, ego_car_.acc_s_mps2, },
      {planned_ego_car_s, planned_ego_car.vel_s_mps, planned_ego_car.acc_s_mps2, },
      planning_time_horizon
  );

  std::vector<double> d_coeffs = GetJerkMinimizingTrajectory(
      { ego_car_.d_m, ego_car_.vel_d_mps, ego_car_.acc_d_mps2, },
      { planned_ego_car.d_m, planned_ego_car.vel_d_mps, planned_ego_car.acc_d_mps2, },
      planning_time_horizon
  );

  double t = pp_config_.frequency_s;
  const double t_diff = pp_config_.frequency_s;
  double s_prev = static_cast<double>(ego_car_.s_m);
  double d_prev = ego_car_.d_m;
  double vs_prev = ego_car_.vel_s_mps;
  double vd_prev = ego_car_.vel_d_mps;


  for (int i = 0; i < pp_config_.trajectory_layer_queue_len - next_cars_.size(); ++i) {
    double s = CalcPolynomial(s_coeffs, t);
    double d = CalcPolynomial(d_coeffs, t);
    double vs = Calc1DVelocity(s_prev, s, t_diff);
    double vd = Calc1DVelocity(d_prev, d, t_diff);
    next_cars_.push_front(
        {
          .id = ego_car_.id,
          .state = ego_car_.state,
          .vel_mps = CalcAbsVelocity(vs, vd),
          .time_s = ego_car_.time_s + t,
          .s_m = s,
          .d_m = d,
          .vel_s_mps = vs,
          .vel_d_mps = vd,
          .acc_s_mps2 = Calc1DAcc(vs_prev, vs, t_diff),
          .acc_d_mps2 = Calc1DAcc(vd_prev, vd, t_diff),
        }
    );
    s_prev = s;
    d_prev = d;
    vs_prev = vs;
    vd_prev = vd;

    t += t_diff;
  }


  std::vector<Car> to_return{next_cars_.rbegin(), next_cars_.rbegin() + num_points};
  ego_car_ = to_return[to_return.size() - 1];
  next_cars_.resize(next_cars_.size() - num_points);

  return to_return;
}


void TrajectoryLayer::Initialize(const Car& car)
{
  ego_car_ = car;
  initialized_ = true;
}


TrajectoryLayer::~TrajectoryLayer() = default;

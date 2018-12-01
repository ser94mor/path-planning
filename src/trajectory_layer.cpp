//
// Created by aoool on 15.11.18.
//

#include "trajectory_layer.hpp"

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;


std::vector<double>
TrajectoryLayer::GetJerkMinimizingTrajectory(std::vector<double> start, std::vector<double> end, double t) {
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
  path_planner_config_{config},
  localization_layer_{localization_layer},
  prediction_layer_{prediction_layer},
  behavior_layer_{behavior_layer}
{

}

std::vector< std::vector<double> > TrajectoryLayer::GetTrajectory(const FrenetCar& car) const {
  return std::vector<std::vector<double>>();
}


TrajectoryLayer::~TrajectoryLayer() = default;

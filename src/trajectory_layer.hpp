//
// Created by aoool on 15.11.18.
//

#ifndef PATH_PLANNING_TRAJECTORY_LAYER_HPP
#define PATH_PLANNING_TRAJECTORY_LAYER_HPP

#include <vector>

class TrajectoryLayer {
public:
  TrajectoryLayer();

  virtual ~TrajectoryLayer();

  std::vector<double> GetJerkMinimizingTrajectory(std::vector<double> start, std::vector<double> end, double t);
};


#endif //PATH_PLANNING_TRAJECTORY_LAYER_HPP

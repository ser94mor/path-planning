//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_LOCALIZATION_LAYER_HPP
#define PATH_PLANNING_LOCALIZATION_LAYER_HPP

#include "car.hpp"

#include <vector>



class LocalizationLayer {

public:
  explicit LocalizationLayer(const PathPlannerConfig& config);

  void Update(const std::vector< std::vector<double> >& sensor_fusion, double time);

  std::vector<FrenetCar> GetFrenetCars();

  virtual ~LocalizationLayer();

private:
  const PathPlannerConfig& path_planner_config_;
  std::vector< std::vector<double> > sensor_fusion_;
  double time_;
  std::vector<FrenetCar> cars_;
  bool cars_updated_;
  uint64_t update_cnt_;
};


#endif //PATH_PLANNING_LOCALIZATION_LAYER_HPP

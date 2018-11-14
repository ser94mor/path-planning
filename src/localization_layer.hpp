//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_LOCALIZATION_LAYER_HPP
#define PATH_PLANNING_LOCALIZATION_LAYER_HPP

#include "car.hpp"

#include <vector>



class LocalizationLayer {

public:
  LocalizationLayer();

  void Update(const std::vector< std::vector<double> >& sensor_fusion);

  const std::vector<Car>& GetCars();

  virtual ~LocalizationLayer();

private:
  std::vector< std::vector<double> > sensor_fusion_;
  std::vector<Car> cars_;
  bool cars_updated_;
};


#endif //PATH_PLANNING_LOCALIZATION_LAYER_HPP

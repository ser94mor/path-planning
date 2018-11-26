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

  std::vector<Car> GetCars();
  std::pair< uint64_t, std::vector<Car> > GetUpdateCntCarsPair();

  virtual ~LocalizationLayer();

private:
  std::vector< std::vector<double> > sensor_fusion_;
  std::vector<Car> cars_;
  bool cars_updated_;
  uint64_t update_cnt_;
};


#endif //PATH_PLANNING_LOCALIZATION_LAYER_HPP

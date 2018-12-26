//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_BEHAVIOR_LAYER_HPP
#define PATH_PLANNING_BEHAVIOR_LAYER_HPP

#include "prediction_layer.hpp"
#include "localization_layer.hpp"
#include "path_planner_config.hpp"


class BehaviorLayer {

public:

  BehaviorLayer(const PathPlannerConfig& path_planner_config,
                LocalizationLayer& localization_layer,
                PredictionLayer& prediction_layer);

  virtual ~BehaviorLayer();

  Car Plan(const Car& ego_car) const;

private:

  Car PlanWithNoObstacles(const Car& ego_car, double t) const;
  std::optional<Car> PlanWithObstacles(const Car& ego_car, const std::map<Car, Car>& predictions, double t) const;

  Car PlanForState(const Car& ego_car, const std::map<Car, Car>& predictions) const;

  Car ChooseBestPlannedEgoCar(std::vector<Car>& planned_ego_cars,
                              const Car& cur_ego_car,
                              const std::map<Car, Car>& predictions) const;

  const PathPlannerConfig& pp_config_;
  LocalizationLayer&       localization_layer_;
  PredictionLayer&         prediction_layer_;
};


#endif //PATH_PLANNING_BEHAVIOR_LAYER_HPP

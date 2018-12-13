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

  Car Plan(const Car& ego_car);

private:

  Car PlanForState(State state, const Car& ego_car, const std::map<Car, Car>& predictions);

  Car PlanForKeepLaneState(const Car& ego_car, const std::map<Car, Car>& predictions);
  Car PlanForKeepLaneStateAndNoObstacles(const Car& ego_car);

  Car PlanForPrepareLaneChangeLeftState(const Car& ego_car,
                                              const std::map<Car, Car>& predictions);

  Car PlanForPrepareLaneChangeRightState(const Car& ego_car,
                                               const std::map<Car, Car>& predictions);

  Car PlanForLaneChangeLeftState(const Car& ego_car, const std::map<Car, Car>& predictions);

  Car PlanForLaneChangeRightState(const Car& ego_car, const std::map<Car, Car>& predictions);

  const PathPlannerConfig& pp_config_;
  LocalizationLayer&       localization_layer_;
  PredictionLayer&         prediction_layer_;
};


#endif //PATH_PLANNING_BEHAVIOR_LAYER_HPP

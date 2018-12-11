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

  FrenetCar Plan(const FrenetCar& ego_car);

private:

  FrenetCar PlanForState(State state, const FrenetCar& ego_car, const std::map<FrenetCar, FrenetCar>& predictions);

  FrenetCar PlanForKeepLaneState(const FrenetCar& ego_car, const std::map<FrenetCar, FrenetCar>& predictions);
  FrenetCar PlanForKeepLaneStateAndNoObstacles(const FrenetCar& ego_car);

  FrenetCar PlanForPrepareLaneChangeLeftState(const FrenetCar& ego_car,
                                              const std::map<FrenetCar, FrenetCar>& predictions);

  FrenetCar PlanForPrepareLaneChangeRightState(const FrenetCar& ego_car,
                                               const std::map<FrenetCar, FrenetCar>& predictions);

  FrenetCar PlanForLaneChangeLeftState(const FrenetCar& ego_car, const std::map<FrenetCar, FrenetCar>& predictions);

  FrenetCar PlanForLaneChangeRightState(const FrenetCar& ego_car, const std::map<FrenetCar, FrenetCar>& predictions);

  const PathPlannerConfig& pp_config_;
  LocalizationLayer&       localization_layer_;
  PredictionLayer&         prediction_layer_;
};


#endif //PATH_PLANNING_BEHAVIOR_LAYER_HPP

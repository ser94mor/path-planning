//
// Created by aoool on 28.10.18.
//

#ifndef PATH_PLANNING_CAR_HPP
#define PATH_PLANNING_CAR_HPP

#include "fsm.hpp"
#include "helpers.hpp"
#include "circular_unsigned_double_t.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <optional>
#include <memory>
#include <functional>

class CarBuilder;

struct Car {

private:
  int id_;
  FSM::State state_;
  double time_s_;
  circular_unsigned_double_t s_m_;
  double d_m_;
  double vel_s_mps_;
  double vel_d_mps_;
  double acc_s_mps2_;
  double acc_d_mps2_;

  int prev_current_lane_;

  static const PathPlannerConfig* pp_config_;

  std::vector<Car> Filter(const std::vector<Car>& cars, const std::function<bool(const Car&)>& predicate) const;
  std::vector<Car> CarsInRange(const std::vector<Car>& cars, double lower, double upper) const;

public:
  int Id() const;
  FSM::State State() const;
  double T() const;
  const circular_unsigned_double_t& S() const;
  double D() const;
  double Vs() const;
  double Vd() const;
  double V() const;
  double As() const;
  double Ad() const;
  double A() const;

  double LongitudinalForwardDistanceTo(const Car& car) const;
  double LongitudinalBackwardDistanceTo(const Car& car) const;
  double LateralDistanceTo(const Car& car) const;

  bool IsFrontBufferViolatedBy(const Car& car, double factor = 1.0) const;
  bool IsBackBufferViolatedBy(const Car& car,  double factor = 1.0) const;
  bool IsSideBufferViolatedBy(const Car& car) const;
  bool AreLongitudinalBuffersViolatedBy(const std::vector<Car>& cars, double factor = 1.0) const;

  bool IsInFrontOf(const Car& car) const;
  bool IsBehind(const Car& car) const;

  bool IsInLeftMostLane() const;
  bool IsInRightMostLane() const;

  int CurrentLane() const;
  int IntendedLane() const;
  int FinalLane() const;

  std::vector<Car> CarsInRegionOfInterest(const std::vector<Car>& cars) const;
  std::vector<Car> CarsInLongitudinalBuffers(const std::vector<Car>& cars, double factor = 1.0) const;
  std::vector<Car> CarsInCurrentLane(const std::vector<Car>& cars) const;
  std::vector<Car> CarsInIntendedLane(const std::vector<Car>& cars) const;
  std::vector<Car> CarsInFinalLane(const std::vector<Car>& cars) const;

  std::optional<Car> NearestCarAhead(const std::vector<Car>& cars) const;
  std::optional<Car> NearestCarAheadInCurrentLane(const std::vector<Car>& cars) const;
  std::optional<Car> NearestCarAheadInIntendedLane(const std::vector<Car>& cars) const;
  std::optional<Car> NearestCarAheadInFinalLane(const std::vector<Car>& cars) const;

  std::optional<Car> NearestCarBehind(const std::vector<Car>& cars) const;
  std::optional<Car> NearestCarBehindInCurrentLane(const std::vector<Car>& cars) const;
  std::optional<Car> NearestCarBehindInIntendedLane(const std::vector<Car>& cars) const;
  std::optional<Car> NearestCarBehindInFinalLane(const std::vector<Car>& cars) const;


  std::vector<FSM::State> PossibleNextStates() const;

  /**
 * @brief Transforms data provided by one item from sensor fusion vector to Car.
 * @throw std::domain_error If the car does not keep lane or moves with the acceleration.
 * @return Instance of Car corresponding to this Cartesian car.
 */
  static Car
  FromVectorAssumingConstantVelocityAndLaneKeeping(const std::vector<double>& car_info,
                                                   double time,
                                                   const PathPlannerConfig& config);
  static std::string CarMapToString(const std::map<Car, Car>& cars);
  static void SetPathPlannerConfig(const PathPlannerConfig* pp_config);
  static double MaxVelocity();
  static size_t NumberOfSettableFields();
  static CarBuilder Builder();
  static CarBuilder Builder(const Car& car);

  bool operator<(const Car& rhs) const;

  bool operator>(const Car& rhs) const;

  bool operator<=(const Car& rhs) const;

  bool operator>=(const Car& rhs) const;

  bool operator==(const Car& rhs) const;

  bool operator!=(const Car& rhs) const;

  std::string str() const;

  friend std::ostream& operator<<(std::ostream& os, const Car& car);

  friend class CarBuilder;

};


class CarBuilder {

public:
  CarBuilder();
  explicit CarBuilder(const Car& car);

  CarBuilder& SetId(int id);
  CarBuilder& SetState(FSM::State state);
  CarBuilder& SetTime(double time);
  CarBuilder& SetCoordinateS(circular_unsigned_double_t s);
  CarBuilder& SetCoordinateD(double d);
  CarBuilder& SetVelocityS(double vs);
  CarBuilder& SetVelocityD(double vd);
  CarBuilder& SetAccelerationS(double as);
  CarBuilder& SetAccelerationD(double ad);

  const Car& Build() const;

private:
  Car car_;
  std::vector<bool> set_flags_;
};

#endif //PATH_PLANNING_CAR_HPP

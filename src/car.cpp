#include <utility>

//
// Created by aoool on 14.11.18.
//

#include "car.hpp"
#include "fsm.hpp"

#include <typeinfo>
#include <numeric>
#include <functional>

const PathPlannerConfig* Car::pp_config_ = nullptr;


Car Car::FromVectorAssumingConstantVelocityAndLaneKeeping(const std::vector<double>& car_info,
                                                          const double time,
                                                          const PathPlannerConfig& config)
{
  return Car::Builder()
           .SetId(static_cast<int>(car_info[0]))
           .SetState(FSM::State::KeepLane)
           .SetTime(time)
           .SetCoordinateS(car_info[5])
           .SetCoordinateD(car_info[6])
           .SetVelocityS(CalcAbsVelocity(car_info[3], car_info[4])) // lane keeping
           .SetVelocityD(0.0)                                       //
           .SetAccelerationS(0.0)               // constant velocity
           .SetAccelerationD(0.0)               //
         .Build();
}

double Car::LongitudinalForwardDistanceTo(const Car& car) const {
  return static_cast<double>(car.s_m_ - this->s_m_);
}

double Car::LongitudinalBackwardDistanceTo(const Car& car) const {
  return static_cast<double>(this->s_m_ - car.s_m_);
}

double Car::LateralDistanceTo(const Car& car) const {
  return fabs(this->d_m_ - car.d_m_);
}

void Car::SetPathPlannerConfig(const PathPlannerConfig* pp_config) {
  pp_config_ = pp_config;
}

bool Car::IsFrontBufferViolatedBy(const Car& car, double factor) const {
  if (!pp_config_) {
    throw std::logic_error("Car::pp_config_ pointer must have been initialized by "
                           "Car::SetPathPlannerConfig before");
  }
  return this->LongitudinalForwardDistanceTo(car) < (factor * pp_config_->front_car_buffer_m);
}

bool Car::IsBackBufferViolatedBy(const Car& car, double factor) const {
  if (!pp_config_) {
    throw std::logic_error("Car::pp_config_ pointer must have been initialized by "
                           "Car::SetPathPlannerConfig before");
  }
  return this->LongitudinalBackwardDistanceTo(car) < (factor * pp_config_->back_car_buffer_m);
}

bool Car::IsSideBufferViolatedBy(const Car& car) const {
  if (!pp_config_) {
    throw std::logic_error("Car::pp_config_ pointer must have been initialized by "
                           "Car::SetPathPlannerConfig before");
  }
  return this->LateralDistanceTo(car) < pp_config_->side_car_buffer_m;
}

bool Car::IsInFrontOf(const Car& car) const {
  return (this->s_m_ >= car.s_m_);
}

bool Car::IsBehind(const Car& car) const {
  return (this->s_m_ < car.s_m_);
}

std::vector<Car> Car::CarsInRegionOfInterest(const std::vector<Car>& cars) const {
  std::vector<Car> cars_in_range;
  for (const auto& car: cars) {
    if (car.s_m_ >= (this->s_m_ - pp_config_->region_of_interest_back_m) &&
        car.s_m_ < (this->s_m_ + pp_config_->region_of_interest_front_m)) {
      cars_in_range.push_back(car);
    }
  }

  return cars_in_range;
}


int Car::CurrentLane() const {
  return static_cast<int>(floor(this->d_m_ / pp_config_->lane_width_m));
}


int Car::IntendedLane() const {
  switch (this->state_) {

    case FSM::State::KeepLane:
      return this->CurrentLane();

    case FSM::State::PrepareLaneChangeLeft:
      return this->CurrentLane() - 1;

    case FSM::State::LaneChangeLeft:
      return prev_current_lane_ - 1;

    case FSM::State::PrepareLaneChangeRight:
      return this->CurrentLane() + 1;

    case FSM::State::LaneChangeRight:
      return prev_current_lane_ + 1;

    default:
      std::cerr << __PRETTY_FUNCTION__ << " must handle " << state_ << " state" << std::endl;
      std::exit(EXIT_FAILURE);
  }
}

int Car::FinalLane() const
{
  switch (this->state_) {

    case FSM::State::KeepLane:
    case FSM::State::PrepareLaneChangeLeft:
    case FSM::State::PrepareLaneChangeRight:
      return this->CurrentLane();

    case FSM::State::LaneChangeLeft:
      return prev_current_lane_ - 1;

    case FSM::State::LaneChangeRight:
      return prev_current_lane_ + 1;

    default:
      std::cerr << __PRETTY_FUNCTION__ << " must handle " << state_ << " state" << std::endl;
      std::exit(EXIT_FAILURE);
  }
}

std::vector<Car> Car::CarsInCurrentLane(const std::vector<Car>& cars) const
{
  return Filter(cars, [&cars, this](const Car& car) { return car.CurrentLane() == this->CurrentLane(); });
}

std::vector<Car> Car::CarsInIntendedLane(const std::vector<Car>& cars) const
{
  return Filter(cars, [&cars, this](const Car& car) { return car.CurrentLane() == this->IntendedLane(); });
}

std::vector<Car> Car::CarsInFinalLane(const std::vector<Car>& cars) const
{
  return Filter(cars, [&cars, this](const Car& car) { return car.CurrentLane() == this->FinalLane(); });
}

std::vector<Car>
Car::Filter(const std::vector<Car>& cars, const std::function<bool(const Car&)>& predicate) const
{
  std::vector<Car> filtered_cars;
  std::copy_if(cars.begin(), cars.end(), std::back_inserter(filtered_cars), predicate);
  return filtered_cars;
}

std::optional<Car> Car::NearestCarAhead(const std::vector<Car>& cars) const
{
  std::optional<Car> opt{std::nullopt};

  for (const auto& car : cars) {
    if (car.s_m_ >= this->s_m_ && (!opt.has_value() || opt.value().s_m_ > car.s_m_)) {
      opt = car;
    }
  }

  return opt;
}

std::optional<Car> Car::NearestCarBehind(const std::vector<Car>& cars) const
{
  std::optional<Car> opt{std::nullopt};

  for (const auto& car : cars) {
    if (car.s_m_ < this->s_m_ && (!opt.has_value() || opt.value().s_m_ < car.s_m_)) {
      opt = car;
    }
  }

  return opt;
}

bool Car::operator<(const Car& rhs) const
{
  return (this->id_ < rhs.id_);
}

bool Car::operator>(const Car& rhs) const
{
  return rhs < *this;
}

bool Car::operator<=(const Car& rhs) const
{
  return !(rhs < *this);
}

bool Car::operator>=(const Car& rhs) const
{
  return !(*this < rhs);
}

bool Car::operator==(const Car& rhs) const
{
  return this->id_ == rhs.id_                        &&
         this->state_ == rhs.state_                  &&
         IsEqual(this->time_s_, rhs.time_s_)         &&
         this->s_m_ == rhs.s_m_                      &&
         IsEqual(this->d_m_, rhs.d_m_)               &&
         IsEqual(this->vel_s_mps_, rhs.vel_s_mps_)   &&
         IsEqual(this->vel_d_mps_, rhs.vel_d_mps_)   &&
         IsEqual(this->acc_s_mps2_, rhs.acc_s_mps2_) &&
         IsEqual(this->acc_d_mps2_, rhs.acc_d_mps2_);
}

bool Car::operator!=(const Car& rhs) const
{
  return !(rhs == *this);
}

std::ostream& operator<<(std::ostream& ostream, const Car& car)
{
  ostream << std::fixed
          << "Car{\n"
          << "  .id_         = " << car.id_         << ",\n"
          << "  .state_      = " << car.state_      << ",\n"
          << "  .time_s_     = " << car.time_s_     << ",\n"
          << "  .s_m_        = " << car.s_m_        << ",\n"
          << "  .d_m_        = " << car.d_m_        << ",\n"
          << "  .vel_s_mps_  = " << car.vel_s_mps_  << ",\n"
          << "  .vel_d_mps_  = " << car.vel_d_mps_  << ",\n"
          << "  .acc_s_mps2_ = " << car.acc_s_mps2_ << ",\n"
          << "  .acc_d_mps2_ = " << car.acc_d_mps2_ << ",\n"
          << "}";
  return ostream;
}

double Car::MaxVelocity()
{
  if (!pp_config_) {
    throw std::logic_error("Car::pp_config_ pointer must have been initialized by "
                           "Car::SetPathPlannerConfig before");
  }

  return pp_config_->max_speed_mps;
}

std::vector<FSM::State> Car::PossibleNextStates() const {
  auto possible_next_states = FSM::GetPossibleNextStates(this->state_); // a copy of what is returned
  possible_next_states.erase(std::remove_if(possible_next_states.begin(),
                                            possible_next_states.end(),
                                            [this](enum FSM::State state) {
    return (this->IsInLeftMostLane()  && (state == FSM::State::PrepareLaneChangeLeft  || state == FSM::State::LaneChangeLeft )) ||
           (this->IsInRightMostLane() && (state == FSM::State::PrepareLaneChangeRight || state == FSM::State::LaneChangeRight));
                                            }),
                             possible_next_states.end());

  return possible_next_states;
}

bool Car::IsInLeftMostLane() const
{
  return (this->CurrentLane() == 0);
}

bool Car::IsInRightMostLane() const
{
  return (this->CurrentLane() == (pp_config_->num_lanes - 1));
}

std::optional<Car> Car::NearestCarAheadInCurrentLane(const std::vector<Car>& cars) const {
  auto&& reg_of_interest_cars = this->CarsInRegionOfInterest(cars);
  auto&& in_cur_lane_cars = this->CarsInCurrentLane(reg_of_interest_cars);

  return this->NearestCarAhead(in_cur_lane_cars);
}

std::optional<Car> Car::NearestCarAheadInIntendedLane(const std::vector<Car>& cars) const {
  auto&& reg_of_interest_cars = this->CarsInRegionOfInterest(cars);
  auto&& in_intended_lane_cars = this->CarsInIntendedLane(reg_of_interest_cars);

  return this->NearestCarAhead(in_intended_lane_cars);
}

std::optional<Car> Car::NearestCarAheadInFinalLane(const std::vector<Car>& cars) const {
  auto&& reg_of_interest_cars = this->CarsInRegionOfInterest(cars);
  auto&& in_final_lane_cars = this->CarsInFinalLane(reg_of_interest_cars);

  return this->NearestCarAhead(in_final_lane_cars);
}

std::optional<Car> Car::NearestCarBehindInCurrentLane(const std::vector<Car>& cars) const {
  auto&& reg_of_interest_cars = this->CarsInRegionOfInterest(cars);
  auto&& in_cur_lane_cars = this->CarsInCurrentLane(reg_of_interest_cars);

  return this->NearestCarBehind(in_cur_lane_cars);
}

std::optional<Car> Car::NearestCarBehindInIntendedLane(const std::vector<Car>& cars) const {
  auto&& reg_of_interest_cars = this->CarsInRegionOfInterest(cars);
  auto&& in_intended_lane_cars = this->CarsInIntendedLane(reg_of_interest_cars);

  return this->NearestCarBehind(in_intended_lane_cars);
}


std::optional<Car> Car::NearestCarBehindInFinalLane(const std::vector<Car>& cars) const {
  auto&& reg_of_interest_cars = this->CarsInRegionOfInterest(cars);
  auto&& in_final_lane_cars = this->CarsInFinalLane(reg_of_interest_cars);

  return this->NearestCarBehind(in_final_lane_cars);
}


size_t Car::NumberOfSettableFields() {
  return 9;
}


double Car::V() const {
  return CalcAbsVelocity(this->vel_s_mps_, this->vel_d_mps_);
}

CarBuilder Car::Builder() {
  return CarBuilder();
}

CarBuilder Car::Builder(const Car& car) {
  return CarBuilder(car);
}

int Car::Id() const {
  return id_;
}


FSM::State Car::State() const {
  return state_;
}


double Car::T() const {
  return time_s_;
}

const circular_unsigned_double_t& Car::S() const {
  return s_m_;
}


double Car::D() const {
  return d_m_;
}


double Car::Vs() const {
  return vel_s_mps_;
}


double Car::Vd() const {
  return vel_d_mps_;
}


double Car::As() const {
  return acc_s_mps2_;
}


double Car::Ad() const {
  return acc_d_mps2_;
}


CarBuilder::CarBuilder(): car_{}, set_flags_(Car::NumberOfSettableFields())
{
  std::fill_n(set_flags_.begin(), set_flags_.size(), false);
  car_.prev_current_lane_ = -1;
}


CarBuilder::CarBuilder(const Car& car): car_{car}, set_flags_(Car::NumberOfSettableFields())
{
  std::fill_n(set_flags_.begin(), set_flags_.size(), true);
}


CarBuilder& CarBuilder::SetId(int id)
{
  car_.id_ = id;
  set_flags_[0] = true;
  return *this;
}


CarBuilder& CarBuilder::SetState(FSM::State state)
{
  if ((state == FSM::State::LaneChangeLeft || state == FSM::State::LaneChangeRight) &&
      car_.state_ != FSM::State::LaneChangeLeft && car_.state_ != FSM::State::LaneChangeRight) {
    car_.prev_current_lane_ = car_.CurrentLane();
  }

  car_.state_ = state;
  set_flags_[1] = true;
  return *this;
}


CarBuilder& CarBuilder::SetTime(double time)
{
  car_.time_s_ = time;
  set_flags_[2] = true;
  return *this;
}


CarBuilder& CarBuilder::SetCoordinateS(circular_unsigned_double_t s) {
  car_.s_m_ = s;
  set_flags_[3] = true;
  return *this;
}


CarBuilder& CarBuilder::SetCoordinateD(double d) {
  car_.d_m_ = d;
  set_flags_[4] = true;
  return *this;
}


CarBuilder& CarBuilder::SetVelocityS(double vs) {
  car_.vel_s_mps_ = vs;
  set_flags_[5] = true;
  return *this;
}


CarBuilder& CarBuilder::SetVelocityD(double vd) {
  car_.vel_d_mps_ = vd;
  set_flags_[6] = true;
  return *this;
}


CarBuilder& CarBuilder::SetAccelerationS(double as) {
  car_.acc_s_mps2_ = as;
  set_flags_[7] = true;
  return *this;
}


CarBuilder& CarBuilder::SetAccelerationD(double ad) {
  car_.acc_d_mps2_ = ad;
  set_flags_[8] = true;
  return *this;
}


const Car& CarBuilder::Build() const
{
  bool all_set = std::accumulate(set_flags_.begin(), set_flags_.end(), true, std::logical_and<>());
  if (!all_set) {
    throw std::logic_error("Either all setters in CarBuilder should have been invoked "
                           "or copy constructor should have been used.");
  }

  return car_;
}

//
// Created by aoool on 11.12.18.
//

#include "circular_unsigned_double_t.hpp"

#include <limits>
#include <cmath>

double circular_unsigned_double_t::global_max_value_{std::numeric_limits<double>::max()};


bool circular_unsigned_double_t::operator<(const circular_unsigned_double_t& rhs) const
{
  circular_unsigned_double_t res1{value_ - rhs.value_};
  circular_unsigned_double_t res2{rhs.value_ - value_};

  if (res1 == res2 && res1 != 0.0) {
    // cars are located on the ends of the track diameter
    return (value_ < rhs.value_);
  } else if (res1 == 0.0) {
    return false;
  } else {
    return (res2.value_ < res1.value_);
  }
}


bool circular_unsigned_double_t::operator>(const circular_unsigned_double_t& rhs) const
{
  return rhs < *this;
}


bool circular_unsigned_double_t::operator<=(const circular_unsigned_double_t& rhs) const
{
  return !(rhs < *this);
}


bool circular_unsigned_double_t::operator>=(const circular_unsigned_double_t& rhs) const
{
  return !(*this < rhs);
}


bool circular_unsigned_double_t::operator==(const circular_unsigned_double_t& rhs) const
{
  return fabs(value_ - rhs.value_) < 0.000000001;
}


bool circular_unsigned_double_t::operator!=(const circular_unsigned_double_t& rhs) const
{
  return !(rhs == *this);
}


std::ostream& operator<<(std::ostream& os, const circular_unsigned_double_t& other) {
  os << other.value_;
  return os;
}


circular_unsigned_double_t circular_unsigned_double_t::operator+(const circular_unsigned_double_t& rhs) const {
  return circular_unsigned_double_t{value_ + rhs.value_};
}

circular_unsigned_double_t& circular_unsigned_double_t::operator+=(const circular_unsigned_double_t& rhs) {
  this->value_ = circular_unsigned_double_t{value_ + rhs.value_}.value_;
  return *this;
}


circular_unsigned_double_t circular_unsigned_double_t::operator-(const circular_unsigned_double_t& rhs) const {
  return circular_unsigned_double_t{value_ - rhs.value_};
}


circular_unsigned_double_t& circular_unsigned_double_t::operator-=(const circular_unsigned_double_t& rhs) {
  this->value_ = circular_unsigned_double_t{value_ - rhs.value_}.value_;
  return *this;
}


circular_unsigned_double_t circular_unsigned_double_t::operator*(const circular_unsigned_double_t& rhs) const {
  return circular_unsigned_double_t{value_ * rhs.value_};
}


circular_unsigned_double_t& circular_unsigned_double_t::operator*=(const circular_unsigned_double_t& rhs) {
  this->value_ = circular_unsigned_double_t{value_ * rhs.value_}.value_;
  return *this;
}


circular_unsigned_double_t circular_unsigned_double_t::operator/(const circular_unsigned_double_t& rhs) const {
  return circular_unsigned_double_t{value_ / rhs.value_};
}


circular_unsigned_double_t& circular_unsigned_double_t::operator/=(const circular_unsigned_double_t& rhs) {
  this->value_ = circular_unsigned_double_t{value_ / rhs.value_}.value_;
  return *this;
}


circular_unsigned_double_t::operator double() const {
  return value_;
}


circular_unsigned_double_t::circular_unsigned_double_t(double value) :
    value_{value - floor(value / global_max_value_) * global_max_value_}
{

}


void circular_unsigned_double_t::SetGlobalMaxValue(double value)
{
  global_max_value_ = value;
}


circular_unsigned_double_t& circular_unsigned_double_t::operator=(double rhs)
{
  *this = circular_unsigned_double_t{rhs};
  return *this;
}


circular_unsigned_double_t::circular_unsigned_double_t(): value_{0.0}
{

}


circular_unsigned_double_t& circular_unsigned_double_t::operator=(const circular_unsigned_double_t& rhs) = default;


circular_unsigned_double_t::~circular_unsigned_double_t() = default;


circular_unsigned_double_t::circular_unsigned_double_t(const circular_unsigned_double_t& other) = default;

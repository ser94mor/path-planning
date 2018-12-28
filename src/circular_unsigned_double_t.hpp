//
// Created by aoool on 11.12.18.
//

#ifndef PATH_PLANNING_CIRCULAR_UNSIGNED_DOUBLE_T_HPP
#define PATH_PLANNING_CIRCULAR_UNSIGNED_DOUBLE_T_HPP

#include <cstdlib>
#include <iostream>
#include <limits>

class circular_unsigned_double_t {
public:
  bool operator<(const circular_unsigned_double_t& rhs) const;

  bool operator>(const circular_unsigned_double_t& rhs) const;

  bool operator<=(const circular_unsigned_double_t& rhs) const;

  bool operator>=(const circular_unsigned_double_t& rhs) const;

  bool operator==(const circular_unsigned_double_t& rhs) const;

  bool operator!=(const circular_unsigned_double_t& rhs) const;

  circular_unsigned_double_t  operator+(const circular_unsigned_double_t& rhs) const;
  circular_unsigned_double_t& operator+=(const circular_unsigned_double_t& rhs);

  circular_unsigned_double_t  operator-(const circular_unsigned_double_t& rhs) const;
  circular_unsigned_double_t& operator-=(const circular_unsigned_double_t& rhs);

  circular_unsigned_double_t  operator*(const circular_unsigned_double_t& rhs) const;
  circular_unsigned_double_t& operator*=(const circular_unsigned_double_t& rhs);

  circular_unsigned_double_t  operator/(const circular_unsigned_double_t& rhs) const;
  circular_unsigned_double_t& operator/=(const circular_unsigned_double_t& rhs);

  circular_unsigned_double_t& operator=(double rhs);

  circular_unsigned_double_t& operator=(const circular_unsigned_double_t& rhs);

  explicit operator double() const;

  friend std::ostream& operator<<(std::ostream& os, const circular_unsigned_double_t& other);

  circular_unsigned_double_t(double value);

  circular_unsigned_double_t();

  circular_unsigned_double_t(const circular_unsigned_double_t& other);

  virtual ~circular_unsigned_double_t();

  static void SetGlobalMaxValue(double value);

private:
  double value_;

  static double global_max_value_;;
};

namespace std {

  template <>
  struct hash<circular_unsigned_double_t>
  {
    std::size_t operator()(const circular_unsigned_double_t& k) const
    {
      return std::hash<double>()(static_cast<double>(k));
    }
  };

}

#endif //PATH_PLANNING_CIRCULAR_UNSIGNED_DOUBLE_T_HPP

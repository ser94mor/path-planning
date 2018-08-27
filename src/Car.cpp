//
// Created by aoool on 8/9/18.
//

#include "Car.hpp"

#include <cmath>

Car::Car(int id, double x_m, double y_m, double v_x_mps, double v_y_mps,
         double yaw_rad, double speed_mps, double s_m, double d_m) :
         id(id),
         x_m(x_m),
         y_m(y_m),
         v_x_mps(v_x_mps),
         v_y_mps(v_y_mps),
         yaw_rad(yaw_rad),
         speed_mps(speed_mps),
         s_m(s_m),
         d_m(d_m)
{}

Car::Car(std::vector<double> &car_params) :
         id(static_cast<int>(car_params[0])),
         x_m(car_params[1]),
         y_m(car_params[2]),
         v_x_mps(car_params[3]),
         v_y_mps(car_params[4]),
         yaw_rad(atan2(v_y_mps, v_x_mps)),
         speed_mps(sqrt(pow(v_x_mps,2) + pow(v_y_mps,2))),
         s_m(car_params[5]),
         d_m(car_params[6])
{}

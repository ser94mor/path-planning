//
// Created by aoool on 8/9/18.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>


struct Car
{
  int id;
  double x_m;
  double y_m;
  double v_x_mps;
  double v_y_mps;
  double yaw_rad;
  double speed_mps;
  double s_m;
  double d_m;


public:
  Car(int id, double x_m, double y_m, double v_x_mps, double v_y_mps,
      double yaw_rad, double speed_mps, double s_m, double d_m);

  explicit Car(std::vector<double> &car_params);

};


#endif //PATH_PLANNING_CAR_H

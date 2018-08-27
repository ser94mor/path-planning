//
// Created by aoool on 7/23/18.
//

#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H


#include "vehicle.h"

using namespace std;

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);

float goal_distance_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,  const map<int, vector<Vehicle>> & predictions, map<string, float> & data);

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data);

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane);

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);



#endif //PATH_PLANNING_COST_H

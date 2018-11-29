//
// Created by aoool on 15.11.18.
//

#include "localization_layer.hpp"
#include "car.hpp"

#include <catch.hpp>

static PathPlannerConfig config{
    .frequency_s = 1,
    .min_speed_mps = 2,
    .max_speed_mps = 3,
    .min_acc_mps2 = 4,
    .max_acc_mps2 = 5,
    .min_jerk_mps3 = 6,
    .max_jerk_mps3 = 7,
    .path_len = 8,
    .num_lanes = 9,
    .lane_width_m = 10,
    .max_s_m = 11,
    .behavior_planning_time_horizon_s = 12,
    // in case of Car::FromVector only map waypoints are important
    .map_wps_x_m  = { 1, 2, 3, 4 },
    .map_wps_y_m  = { 1, 2, 3, 4 },
    .map_wps_s_m  = { sqrt(2), sqrt(8), sqrt(18), sqrt(32) },
    .map_wps_dx_m = { sqrt(0.5)+1, sqrt(0.5)+2, sqrt(0.5)+3, sqrt(0.5)+4 },
    .map_wps_dy_m = { -sqrt(0.5)+3, -sqrt(0.5)+6, -sqrt(0.5)+9, -sqrt(0.5)+12 },
    .spline_s_x = {},
    .spline_s_y = {},
    .spline_s_dx = {},
    .spline_s_dy = {},
};

TEST_CASE("LocalizationLayer::GetCars", "[localization_layer]") {
  config.InitSplines();
  
  LocalizationLayer localization_layer{config};

  std::vector<double> car_vect_1{ 12 /*id*/, 37.9 /*x*/, -49.4 /*y*/, -1.9 /*vx*/, 6.5 /*vy*/, 127.8 /*s*/, 0.45 /*d*/};
  std::vector<double> car_vect_2{ 28 /*id*/, 33.3 /*x*/, 555.0 /*y*/, 4.5 /*vx*/, -0.97 /*vy*/, 12.7 /*s*/, 5.33 /*d*/};

  SECTION("LocalizationLayer::GetCars returns zero length cars array when zero length sensor_fusion array provided") {
    REQUIRE( localization_layer.GetCars().empty() );
    localization_layer.Update({});
    REQUIRE( localization_layer.GetCars().empty() );
  }

  SECTION("LocalizationLayer::GetCars returns cars array of length 1 when sensor_fusion array with 1 entry provided") {
    localization_layer.Update({car_vect_1});
    REQUIRE( localization_layer.GetCars().size() == 1 );
    REQUIRE( localization_layer.GetCars()[0] == Car::FromVector(car_vect_1, config) );

    localization_layer.Update({car_vect_2});
    REQUIRE( localization_layer.GetCars().size() == 1 );
    REQUIRE( localization_layer.GetCars()[0] == Car::FromVector(car_vect_2, config) );
  }

  SECTION("LocalizationLayer::GetCars returns cars array of len. 2 when sensor_fusion array with 2 entries provided") {
    localization_layer.Update({car_vect_1, car_vect_2});
    REQUIRE( localization_layer.GetCars().size() == 2 );
    REQUIRE( localization_layer.GetCars()[0] == Car::FromVector(car_vect_1, config) );
    REQUIRE( localization_layer.GetCars()[1] == Car::FromVector(car_vect_2, config) );

    localization_layer.Update({car_vect_2, car_vect_1});
    REQUIRE( localization_layer.GetCars().size() == 2 );
    REQUIRE( localization_layer.GetCars()[0] == Car::FromVector(car_vect_2, config) );
    REQUIRE( localization_layer.GetCars()[1] == Car::FromVector(car_vect_1, config) );
  }
}

TEST_CASE("LocalizationLayer::GetUpdateCntCarsPair", "[localization_layer]") {
  config.InitSplines();
  
  LocalizationLayer localization_layer{config};

  std::vector<double> car_vect_1{ 12 /*id*/, 37.9 /*x*/, -49.4 /*y*/, -1.9 /*vx*/, 6.5 /*vy*/, 127.8 /*s*/, 0.45 /*d*/};
  std::vector<double> car_vect_2{ 28 /*id*/, 33.3 /*x*/, 555.0 /*y*/, 4.5 /*vx*/, -0.97 /*vy*/, 12.7 /*s*/, 5.33 /*d*/};

  SECTION("LocalizationLayer::GetUpdateCntCarsPair returns zero length cars array "
          "when zero length sensor_fusion array provided") {
    auto cnt_cars_pair = localization_layer.GetUpdateCntCarsPair();
    REQUIRE( cnt_cars_pair.second.empty() );
    REQUIRE( cnt_cars_pair.first == 1 );
    localization_layer.Update({});
    cnt_cars_pair = localization_layer.GetUpdateCntCarsPair();
    REQUIRE( cnt_cars_pair.second.empty() );
    REQUIRE( cnt_cars_pair.first == 2 );
  }

  SECTION("LocalizationLayer::GetUpdateCntCarsPair returns cars array of length 1 "
          "when sensor_fusion array with 1 entry provided") {
    localization_layer.Update({car_vect_1});
    auto cnt_car_pair = localization_layer.GetUpdateCntCarsPair();
    REQUIRE( cnt_car_pair.second.size() == 1 );
    REQUIRE( cnt_car_pair.second[0] == Car::FromVector(car_vect_1, config) );
    REQUIRE( cnt_car_pair.first == 1);

    localization_layer.Update({car_vect_2});
    cnt_car_pair = localization_layer.GetUpdateCntCarsPair();
    REQUIRE( cnt_car_pair.second.size() == 1 );
    REQUIRE( cnt_car_pair.second[0] == Car::FromVector(car_vect_2, config) );
    REQUIRE( cnt_car_pair.first == 2 );
  }

  SECTION("LocalizationLayer::GetUpdateCntCarsPair returns cars array of len. 2 "
          "when sensor_fusion array with 2 entries provided") {
    localization_layer.Update({car_vect_1, car_vect_2});
    auto cnt_car_pair = localization_layer.GetUpdateCntCarsPair();
    REQUIRE( cnt_car_pair.second.size() == 2 );
    REQUIRE( cnt_car_pair.second[0] == Car::FromVector(car_vect_1, config) );
    REQUIRE( cnt_car_pair.second[1] == Car::FromVector(car_vect_2, config) );
    REQUIRE( cnt_car_pair.first == 1 );

    localization_layer.Update({car_vect_2, car_vect_1});
    cnt_car_pair = localization_layer.GetUpdateCntCarsPair();
    REQUIRE( cnt_car_pair.second.size() == 2 );
    REQUIRE( cnt_car_pair.second[0] == Car::FromVector(car_vect_2, config) );
    REQUIRE( cnt_car_pair.second[1] == Car::FromVector(car_vect_1, config) );
    REQUIRE( cnt_car_pair.first == 2 );
  }
}

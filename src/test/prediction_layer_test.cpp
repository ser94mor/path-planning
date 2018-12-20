//
// Created by aoool on 02.12.18.
//

#include "prediction_layer.hpp"
#include "car.hpp"

#include <catch.hpp>

static PathPlannerConfig config{
    .frequency_s = 0.02,
    .min_speed_mps = 2,
    .max_speed_mps = 3,
    .min_acc_mps2 = 4,
    .max_acc_mps2 = 5,
    .min_jerk_mps3 = 6,
    .max_jerk_mps3 = 7,
    .path_len = 8,
    .trajectory_layer_queue_len = 9,
    .num_lanes = 10,
    .lane_width_m = 11,
    .max_s_m = 12,
    .behavior_planning_time_horizon_s = 13,
    .front_car_buffer_m = 5.0,
    .back_car_buffer_m = 0.5,
    .side_car_buffer_m = 1.0,
    .map_wps_x_m  = {        0.0,          1.0,          2.0,          3.0,           4.0, },
    .map_wps_y_m  = {        0.0,          1.0,          2.0,          3.0,           4.0, },
    .map_wps_s_m  = {        0.0,      sqrt(2),      sqrt(8),     sqrt(18),      sqrt(32), },
    .map_wps_dx_m = {  sqrt(0.5),  sqrt(0.5)+1,  sqrt(0.5)+2,  sqrt(0.5)+3,   sqrt(0.5)+4, },
    .map_wps_dy_m = { -sqrt(0.5), -sqrt(0.5)+3, -sqrt(0.5)+6, -sqrt(0.5)+9, -sqrt(0.5)+12, },
    .spline_s_x = {},
    .spline_s_y = {},
    .spline_s_dx = {},
    .spline_s_dy = {},
};

TEST_CASE("PredictionLayer::GetPredictionForCar", "[prediction_layer]")
{
  config.InitSplines();

  LocalizationLayer localization_layer{config};

  std::vector<double> car_vect_1{ 12 /*id*/, 1.0 /*x*/, 0.0 /*y*/, 1.0 /*vx*/, 1.0 /*vy*/, sqrt(2.0)/2.0 /*s*/, sqrt(2.0)/2.0 /*d*/};

  localization_layer.Update({car_vect_1}, 1.0);
  auto car_1 = localization_layer.GetCars()[0];

  PredictionLayer prediction_layer{config, localization_layer};

  SECTION("PredictionLayer::GetPredictionForCar returns expected predictions")
  {

    auto prediction = prediction_layer.GetPredictionForCar(car_1, 1.0);
    Car expected_car{
      Car::Builder()
        .SetId(12)
        .SetState(car_1.State())
        .SetTime(2.0)
        .SetCoordinateS(3.0 * sqrt(2.0) / 2.0)
        .SetCoordinateD(sqrt(2.0) / 2.0)
        .SetVelocityS(sqrt(2.0))
        .SetVelocityD(0.0)
        .SetAccelerationS(0.0)
        .SetAccelerationD(0.0)
      .Build()
    };
    REQUIRE( prediction.first == car_1 );
    REQUIRE( prediction.second == expected_car );
  }
}

TEST_CASE("PredictionLayer::GetPredictionsForCars", "[prediction_layer]")
{
  config.InitSplines();

  LocalizationLayer localization_layer{config};

  std::vector<double> car_vect_1{ 12 /*id*/, 1.0 /*x*/, 0.0 /*y*/, 1.0 /*vx*/, 1.0 /*vy*/, 0.5*sqrt(2.0) /*s*/,  sqrt(2.0)/2.0 /*d*/};
  std::vector<double> car_vect_2{ 28 /*id*/, 1.0 /*x*/, 2.0 /*y*/, 1.0 /*vx*/, 1.0 /*vy*/, 1.5*sqrt(2.0) /*s*/, -sqrt(2.0)/2.0 /*d*/};

  PredictionLayer prediction_layer{config, localization_layer};

  SECTION("PredictionLayer::GetPredictionsForCars should return "
          "expected number of cars and predictions are correct")
  {
    localization_layer.Update({car_vect_1}, 1.0);
    auto car_1 = localization_layer.GetCars()[0];

    Car expected_car_1{
      Car::Builder()
        .SetId(12)
        .SetState(car_1.State())
        .SetTime(2.0)
        .SetCoordinateS(3.0 * sqrt(2.0) / 2.0)
        .SetCoordinateD(sqrt(2.0) / 2.0)
        .SetVelocityS(sqrt(2.0))
        .SetVelocityD(0.0)
        .SetAccelerationS(0.0)
        .SetAccelerationD(0.0)
      .Build()
    };

    auto predictions = prediction_layer.GetPredictionsForCars({car_1}, 1.0);
    REQUIRE( predictions.size() == 1 );
    REQUIRE( predictions.at(car_1) == expected_car_1 );

    localization_layer.Update({car_vect_1, car_vect_2}, 1.0);
    car_1 = localization_layer.GetCars()[0];
    auto car_2 = localization_layer.GetCars()[1];
    predictions = prediction_layer.GetPredictionsForCars(localization_layer.GetCars(), 1.0);

    Car expected_car_2{
      Car::Builder()
        .SetId(28)
        .SetState(car_1.State())
        .SetTime(2.0)
        .SetCoordinateS(2.5 * sqrt(2.0))
        .SetCoordinateD(-sqrt(2.0) / 2.0)
        .SetVelocityS(sqrt(2.0))
        .SetVelocityD(0.0)
        .SetAccelerationS(0.0)
        .SetAccelerationD(0.0)
      .Build()
    };

    REQUIRE( predictions.size() == 2 );
    REQUIRE( predictions.at(car_1) == expected_car_1);
    REQUIRE( predictions.at(car_2) == expected_car_2);
  }
}

TEST_CASE("PredictionLayer::GetPredictions", "[prediction_layer]")
{
  config.InitSplines();

  LocalizationLayer localization_layer{config};

  std::vector<double> car_vect_1{ 12 /*id*/, 1.0 /*x*/, 0.0 /*y*/, 1.0 /*vx*/, 1.0 /*vy*/, 0.5*sqrt(2.0) /*s*/,  sqrt(2)/2.0 /*d*/};
  std::vector<double> car_vect_2{ 28 /*id*/, 1.0 /*x*/, 2.0 /*y*/, 1.0 /*vx*/, 1.0 /*vy*/, 1.5*sqrt(2.0) /*s*/, -sqrt(2)/2.0 /*d*/};

  PredictionLayer prediction_layer{config, localization_layer};

  SECTION("PredictionLayer::GetPredictions should return expected number of cars and predictions are correct")
  {
    localization_layer.Update({car_vect_1}, 1.0);
    auto car_1 = localization_layer.GetCars()[0];

    Car expected_car_1{
      Car::Builder()
        .SetId(12)
        .SetState(car_1.State())
        .SetTime(2.0)
        .SetCoordinateS(1.5 * sqrt(2.0))
        .SetCoordinateD(sqrt(2.0) / 2.0)
        .SetVelocityS(sqrt(2.0))
        .SetVelocityD(0.0)
        .SetAccelerationS(0.0)
        .SetAccelerationD(0.0)
      .Build()
    };

    auto predictions = prediction_layer.GetPredictions(1.0, 1.0);
    REQUIRE( predictions.size() == 1 );
    REQUIRE( predictions.at(car_1) == expected_car_1 );

    localization_layer.Update({car_vect_1, car_vect_2}, 1.0);
    car_1 = localization_layer.GetCars()[0];
    auto car_2 = localization_layer.GetCars()[1];
    predictions = prediction_layer.GetPredictions(1.0, 1.0);

    Car expected_car_2{
      Car::Builder()
        .SetId(28)
        .SetState(car_1.State())
        .SetTime(2.0)
        .SetCoordinateS(2.5 * sqrt(2.0))
        .SetCoordinateD(-sqrt(2.0) / 2.0)
        .SetVelocityS(sqrt(2.0))
        .SetVelocityD(0.0)
        .SetAccelerationS(0.0)
        .SetAccelerationD(0.0)
      .Build()
    };

    REQUIRE( predictions.size() == 2 );
    REQUIRE( predictions.at(car_1) == expected_car_1 );
    REQUIRE( predictions.at(car_2) == expected_car_2 );

    predictions = prediction_layer.GetPredictions(1.0, 1.0);
    REQUIRE( predictions.size() == 2 );
    REQUIRE( predictions.at(car_1) == expected_car_1 );
    REQUIRE( predictions.at(car_2) == expected_car_2 );


    localization_layer.Update({car_vect_1, car_vect_2}, 2.0);
    predictions = prediction_layer.GetPredictions(2.0, 3.0);

    Car expected_car_3{
      Car::Builder()
        .SetId(12)
        .SetState(car_1.State())
        .SetTime(5.0)
        .SetCoordinateS(3.5 * sqrt(2.0))
        .SetCoordinateD(sqrt(2.0) / 2.0)
        .SetVelocityS(sqrt(2.0))
        .SetVelocityD(0.0)
        .SetAccelerationS(0.0)
        .SetAccelerationD(0.0)
      .Build()
    };

    Car expected_car_4{
      Car::Builder()
        .SetId(28)
        .SetState(car_1.State())
        .SetTime(5.0)
        .SetCoordinateS(4.5 * sqrt(2.0))
        .SetCoordinateD(-sqrt(2.0) / 2.0)
        .SetVelocityS(sqrt(2.0))
        .SetVelocityD(0.0)
        .SetAccelerationS(0.0)
        .SetAccelerationD(0.0)
      .Build()
    };

    REQUIRE( predictions.size() == 2 );
    REQUIRE( predictions.at(expected_car_1) == expected_car_3 );
    REQUIRE( predictions.at(expected_car_2) == expected_car_4 );
  }
}

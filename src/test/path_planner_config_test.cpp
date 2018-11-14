//
// Created by aoool on 12.11.18.
//

#include "path_planner.hpp"
#include "helpers.hpp"
#include "temp_file.hpp"

#include <catch.hpp>

TEST_CASE("PathPlannerConfig::FromFile", "[path_planner_config]") {

  // create path planner json configuration file
  TempFile pp_temp{};
  pp_temp.GetOfstream()
              << "{\n"
                 "  \"frequency_s\"   :      0.020,\n"
                 "  \"min_speed_mph\" :      0.000,\n"
                 "  \"max_speed_mph\" :     49.000,\n"
                 "  \"min_acc_mps2\"  :    -10.000,\n"
                 "  \"max_acc_mps2\"  :     10.000,\n"
                 "  \"min_jerk_mps3\" :    -10.000,\n"
                 "  \"max_jerk_mps3\" :     10.000,\n"
                 "  \"path_len\"      :         50,\n"
                 "  \"num_lanes\"     :          3,\n"
                 "  \"lane_width_m\"  :      4.000,\n"
                 "  \"max_s_m\"       :   6945.554\n"
                 "}";
  pp_temp.GetOfstream().flush();

  // create test highway map csv file
  TempFile map_temp{};
  map_temp.GetOfstream()
               << "784.6001 1135.571 0 -0.02359831 -0.9997216\n"
                  "815.2679 1134.93 30.6744785309 -0.01099479 -0.9999396";
  map_temp.GetOfstream().flush();

  SECTION("PathPlannerConfig::FromFile creates a valid PathPlannerConfig from valid config. files") {
    PathPlannerConfig config = PathPlannerConfig::FromFile(pp_temp.GetName(), map_temp.GetName());

    REQUIRE(config.frequency_s == Approx(0.02));
    REQUIRE(config.min_speed_mps == Approx(0.0));
    REQUIRE(config.max_speed_mps == Approx(mph_to_mps(49.0)));
    REQUIRE(config.min_acc_mps2 == Approx(-10.0));
    REQUIRE(config.max_acc_mps2 == Approx(10.0));
    REQUIRE(config.min_jerk_mps3 == Approx(-10.0));
    REQUIRE(config.max_jerk_mps3 == Approx(10.0));
    REQUIRE(config.path_len == 50);
    REQUIRE(config.num_lanes == 3);
    REQUIRE(config.lane_width_m == Approx(4.0));
    REQUIRE(config.max_s_m == Approx(6945.554));

    REQUIRE(config.map_waypoints_x_m.size() == 2);
    REQUIRE(config.map_waypoints_y_m.size() == 2);
    REQUIRE(config.map_waypoints_s_m.size() == 2);
    REQUIRE(config.map_waypoints_dx_m.size() == 2);
    REQUIRE(config.map_waypoints_dy_m.size() == 2);

    REQUIRE(config.map_waypoints_x_m[0] == Approx(784.6001));
    REQUIRE(config.map_waypoints_x_m[1] == Approx(815.2679));

    REQUIRE(config.map_waypoints_y_m[0] == Approx(1135.571));
    REQUIRE(config.map_waypoints_y_m[1] == Approx(1134.93));

    REQUIRE(config.map_waypoints_s_m[0] == Approx(0.0));
    REQUIRE(config.map_waypoints_s_m[1] == Approx(30.6744785309));

    REQUIRE(config.map_waypoints_dx_m[0] == Approx(-0.02359831));
    REQUIRE(config.map_waypoints_dx_m[1] == Approx(-0.01099479));

    REQUIRE(config.map_waypoints_dy_m[0] == Approx(-0.9997216));
    REQUIRE(config.map_waypoints_dy_m[1] == Approx(-0.9999396));
  }

}

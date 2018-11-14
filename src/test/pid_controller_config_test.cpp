//
// Created by aoool on 14.11.18.
//

#include "path_planner.hpp"
#include "helpers.hpp"
#include "temp_file.hpp"

#include <catch.hpp>


TEST_CASE("PIDControllerConfig::FromFile", "[pid_controller_config]") {

  // create path planner json configuration file
  TempFile pid_temp{};
  pid_temp.GetOfstream()
      << "{\n"
         "  \"Kp_initial\": 0.1,\n"
         "  \"Ki_initial\": 0.2,\n"
         "  \"Kd_initial\": 3.3,\n"
         "  \"twiddle_dKp_initial\": 6.1,\n"
         "  \"twiddle_dKi_initial\": 10.0,\n"
         "  \"twiddle_dKd_initial\": -11.9,\n"
         "  \"delay_before_calc_tot_error\": 5,\n"
         "  \"frequency_of_coeff_tuning\": 3,\n"
         "  \"stop_threshold\": 6.0\n"
         "}";
  pid_temp.GetOfstream().flush();

  SECTION("PIDControllerConfig::FromFile creates a valid PIDControllerConfig from valid config. files") {
    PIDControllerConfig config = PIDControllerConfig::FromFile(pid_temp.GetName());

    REQUIRE(config.Kp_initial == Approx(0.1));
    REQUIRE(config.Ki_initial == Approx(0.2));
    REQUIRE(config.Kd_initial == Approx(3.3));
    REQUIRE(config.twiddle_dKp_initial == Approx(6.1));
    REQUIRE(config.twiddle_dKi_initial == Approx(10.0));
    REQUIRE(config.twiddle_dKd_initial == Approx(-11.9));
    REQUIRE(config.delay_before_calc_tot_error == 5);
    REQUIRE(config.frequency_of_coeff_tuning == 3);
    REQUIRE(config.stop_threshold == Approx(6.0));

  }

}


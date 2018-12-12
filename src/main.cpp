#include "helpers.hpp"
#include "car.hpp"
#include "fsm.hpp"
#include "path_planner.hpp"

#include <fstream>
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <limits>
#include <Eigen/Core>
#include <Eigen/QR>
#include <json.hpp>



using namespace std;

// for convenience
using json = nlohmann::json;

const char* kDefaultPathPlannerConfigFile{"../data/path_planner_config.json"};
const char* kDefaultPIDControllerConfigFile{"../data/pid_controller_config.json"};
const char* kDefaultHighwayMapFile{"../data/highway_map.csv"};


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of('[');
  auto b2 = s.find_first_of('}');
  if (found_null != string::npos)
  {
    return "";
  } else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main(int argc, char* argv[])
{
  //////
  // initialize file names which provide information about the environment and configuration information
  //////
  char* path_planner_config_file = const_cast<char*>(kDefaultPathPlannerConfigFile);
  char* pid_controller_config_file = const_cast<char*>(kDefaultPIDControllerConfigFile);
  char* highway_map_file = const_cast<char*>(kDefaultHighwayMapFile);
  if (argc == 4) {
    path_planner_config_file = argv[1];
    pid_controller_config_file = argv[2];
    highway_map_file = argv[3];
  } else if (argc != 1) {
    std::cerr << "either no arguments or 3 arguments should be provided---"
                 "path planner and pid controller configuration files as well as highway map file."
              << std::endl;
    std::exit(1);
  }

  //////
  // read configuration files
  //////
  PathPlannerConfig path_planner_config = PathPlannerConfig::FromFile(path_planner_config_file, highway_map_file);
  circular_unsigned_double_t::SetGlobalMaxValue(path_planner_config.max_s_m);
  FrenetCar::SetPathPlannerConfig(&path_planner_config);

  PIDControllerConfig pid_controller_config = PIDControllerConfig::FromFile(pid_controller_config_file);

  uWS::Hub h;

  PathPlanner path_planner(path_planner_config, pid_controller_config);
  double global_time_s = 0.0;

  h.onMessage(
      [&path_planner, &global_time_s](
          uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
          uWS::OpCode opCode)
      {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length > 2 && data[0] == '4' && data[1] == '2')
        {

          auto s = hasData(data);

          if (!s.empty())
          {
            auto j = json::parse(s);

            string event = j[0].get<string>();

            if (event == "telemetry")
            {
              // j[1] is the data JSON object

              // Main car's localization Data
              double car_x = j[1]["x"];
              double car_y = j[1]["y"];
              double car_s = j[1]["s"];
              double car_d = j[1]["d"];
              double car_yaw = j[1]["yaw"];
              double car_speed = j[1]["speed"];

              // Previous path data given to the Planner
              std::vector<double> previous_path_x = j[1]["previous_path_x"];
              std::vector<double> previous_path_y = j[1]["previous_path_y"];
              // Previous path's end s and d values
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];

              // Sensor Fusion Data, a list of all other cars on the same side of the road.
              std::vector<std::vector<double> > sensor_fusion = j[1]["sensor_fusion"];

              json msgJson;


              // some values need to be converted into International System of Units
              double vel_mps = MphToMps(car_speed);
              double yaw_rad = DegToRad(car_yaw);

              double vel_x_mps = vel_mps * cos(yaw_rad);
              double vel_y_mps = vel_mps * sin(yaw_rad);

              auto frenet_vel = GetFrenetSpeed(car_s, car_d, car_x, car_y, vel_x_mps, vel_y_mps,
                                               path_planner.GetPathPlannerConfig());
              double vel_s = frenet_vel[0];
              double vel_d = frenet_vel[1];



              FrenetCar car = {
                .id = -1,
                .state = State::KeepLane,
                .vel_mps = vel_mps,
                .time_s = global_time_s,
                .s_m = car_s,
                .d_m = car_d,
                .vel_s_mps = 0.0,       // unknown here; 0.0 only for the first time, when car is not moving
                .vel_d_mps = 0.0,       //
                .acc_s_mps2 = 0.0,      // unknown here; 0.0 only for the first time, when car is not moving
                .acc_d_mps2 = 0.0,      //
              };

              std::vector< std::vector<double> > next_coords =
                  path_planner.GetNextXYTrajectories(car, previous_path_x, previous_path_y, sensor_fusion);

              msgJson["next_x"] = next_coords[0];
              msgJson["next_y"] = next_coords[1];

              auto msg = "42[\"control\"," + msgJson.dump() + "]";

              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              
              global_time_s += path_planner.GetPathPlannerConfig().frequency_s;

            }
          } else
          {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t)
                  {
                    const std::string s = "<h1>Hello world!</h1>";
                    if (req.getUrl().valueLength == 1)
                    {
                      res->end(s.data(), s.length());
                    } else
                    {
                      // i guess this should be done more gracefully?
                      res->end(nullptr, 0);
                    }
                  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                 {
                   std::cout << "Connected!!!" << std::endl;
                 });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length)
                    {
                      ws.close();
                      std::cout << "Disconnected" << std::endl;
                    });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  } else
  {
    std::cerr << "Failed to listen to port" << std::endl;

    return -1;
  }
  h.run();
}

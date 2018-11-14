project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

include_directories( src/ src/Eigen-3.3 src/test )

set(sources src/main.cpp

            src/path_planner.cpp
            src/path_planner.hpp

            src/pid_controller.cpp
            src/pid_controller.hpp

            src/behavior_layer.cpp
            src/behavior_layer.hpp

            src/prediction_layer.cpp
            src/prediction_layer.hpp

            src/localization_layer.cpp
            src/localization_layer.hpp

            src/car.hpp

            src/state.hpp

            src/speed_controller.cpp
            src/speed_controller.hpp

            src/path_planner_config.cpp
            src/path_planner_config.hpp

            src/pid_controller_config.cpp
            src/pid_controller_config.hpp src/car.cpp)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_library(Catch INTERFACE)
add_executable( path_planning_test
        src/test/state_test.cpp src/state.hpp
        src/test/helpers_test.cpp src/helpers.hpp
        src/test/car_test.cpp src/car.hpp src/car.cpp
        src/test/path_planner_config_test.cpp src/path_planner_config.cpp src/path_planner_config.hpp
        src/test/pid_controller_config_test.cpp src/pid_controller_config.cpp src/pid_controller_config.cpp
        src/test/localization_layer_test.cpp src/localization_layer.cpp src/localization_layer.hpp
        src/test/test.cpp src/test/temp_file.cpp src/test/temp_file.hpp)
target_link_libraries(path_planning_test Catch)


add_executable(path_planning ${sources})

enable_testing()
add_test( NAME path_planning_test COMMAND path_planning_test )

target_link_libraries(path_planning z ssl uv uWS)

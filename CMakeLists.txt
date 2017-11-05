cmake_minimum_required (VERSION 3.0)
project (follow_me)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INLCUDE_DIRS})
include_directories("include/")

add_library(ekf src/track/ekf.cpp)
add_library(vehicle src/simulator/vehicle.cpp)

add_executable(car_sim_engine src/car_sim_engine.cpp)

target_link_libraries(car_sim_engine ekf vehicle)

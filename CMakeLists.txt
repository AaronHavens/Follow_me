cmake_minimum_required (VERSION 3.0)
project (follow_me)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INLCUDE_DIRS})
add_executable(car_sim_engine src/car_sim_engine.cpp)

cmake_minimum_required (VERSION 3.0)
project (follow_me)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INLCUDE_DIRS}
			include
			src/qpOASES/include
)

set(QPSOURCE
	src/qpOASES/src/BLASReplacement.cpp
  	src/qpOASES/src/Constraints.cpp
  	src/qpOASES/src/Indexlist.cpp
  	src/qpOASES/src/Matrices.cpp
  	src/qpOASES/src/Options.cpp
  	src/qpOASES/src/QProblemB.cpp
  	src/qpOASES/src/SubjectTo.cpp
  	src/qpOASES/src/Bounds.cpp
  	src/qpOASES/src/Flipper.cpp
  	src/qpOASES/src/LAPACKReplacement.cpp
  	src/qpOASES/src/MessageHandling.cpp
  	src/qpOASES/src/OQPinterface.cpp 
  	src/qpOASES/src/QProblem.cpp
  	src/qpOASES/src/SQProblem.cpp
	src/qpOASES/src/Utils.cpp)

add_library(ekf src/tracking/ekf.cpp)
add_library(vehicle src/simulator/vehicle.cpp)
add_library(error_utils src/control/error_utils.cpp)

add_executable(car_sim_engine src/car_sim_engine.cpp)
add_executable(qp_example ${QPSOURCE} src/control/qp_example.cpp)
target_link_libraries(ekf vehicle)
target_link_libraries(car_sim_engine ekf vehicle error_utils)

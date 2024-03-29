cmake_minimum_required(VERSION 3.14)
project(mpc_lateral_controller)

list(APPEND CMAKE_PREFIX_PATH "/home/yck/Desktop/GITHUB/CppTest/Cpp_Gtest_Projects/Project_5/autoware_cmake/install/autoware_cmake/share/autoware_cmake/cmake")


#find_package(autoware_cmake REQUIRED)
find_package(autoware_cmake CONFIG REQUIRED)
autoware_package()

# Include the directory containing "tier4_autoware_utils/geometry"
#include_directories(/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5)

include_directories(/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/autoware_auto_control_msgs/include/autoware_auto_control_msgs)

include_directories(/home/eby/Desktop/GITHUB/Cpp_Gtest_Projects/Project_5/mpc_lateral_controller/include)



ament_auto_add_library(steering_offset_lib SHARED
  src/steering_offset/steering_offset.cpp
)



set(MPC_LAT_CON_LIB ${PROJECT_NAME}_lib)
ament_auto_add_library(${MPC_LAT_CON_LIB} SHARED
  src/mpc_lateral_controller.cpp
  src/lowpass_filter.cpp
  src/steering_predictor.cpp
  src/mpc.cpp
  src/mpc_trajectory.cpp
  src/mpc_utils.cpp
  src/qp_solver/qp_solver_osqp.cpp
  src/qp_solver/qp_solver_unconstraint_fast.cpp
  src/vehicle_model/vehicle_model_bicycle_dynamics.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_interface.cpp
)
target_link_libraries(${MPC_LAT_CON_LIB} steering_offset_lib)

if(BUILD_TESTING)
  set(TEST_LAT_SOURCES
    test/test_mpc.cpp
    test/test_mpc_utils.cpp
    test/test_lowpass_filter.cpp
  )
  set(TEST_LATERAL_CONTROLLER_EXE test_lateral_controller)
  ament_add_ros_isolated_gtest(${TEST_LATERAL_CONTROLLER_EXE} ${TEST_LAT_SOURCES})
  target_link_libraries(${TEST_LATERAL_CONTROLLER_EXE} ${MPC_LAT_CON_LIB})
endif()

ament_auto_package(INSTALL_TO_SHARE
  param
)

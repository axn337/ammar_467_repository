cmake_minimum_required(VERSION 2.8.3)
project(ps8_axn337)

find_package(catkin_simple REQUIRED)


catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   
cs_add_library(ps8_traj_builder src/ps8_traj_builder.cpp)   

# Executables
cs_add_executable(ps8_lidar_alarm src/ps8_lidar_alarm.cpp)
cs_add_executable(ps8_pub_des_state src/ps8_pub_des_state_main.cpp src/ps8_pub_des_state.cpp)
cs_add_executable(ps8_pub_des_state_path_client src/ps8_pub_des_state_path_client.cpp)
cs_add_executable(ps8_open_loop_controller src/ps8_open_loop_controller.cpp)
cs_add_executable(ps8_lin_steering src/ps8_lin_steering.cpp)


#cs_add_executable(lidar_alarm2 src/lidar_alarm2.cpp)
# target_link_library(example my_lib)
target_link_libraries(ps8_pub_des_state ps8_traj_builder)

cs_install()
cs_export()
    

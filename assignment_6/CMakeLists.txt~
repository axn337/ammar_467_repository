cmake_minimum_required(VERSION 2.8.3)
project(assignment_6)

find_package(catkin_simple REQUIRED)


catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_libraries(my_lib src/my_lib.cpp)   
cs_add_library(ps6_traj_builder src/ps6_traj_builder.cpp)   

# Executables
cs_add_executable(ps6_lidar_alarm src/ps6_lidar_alarm.cpp)
#cs_add_executable(ps6_pub_des_state src/ps6_pub_des_state.cpp)

#cs_add_executable(lidar_alarm2 src/lidar_alarm2.cpp)
# target_link_library(example my_lib)
#target_link_libraries(ps6_pub_des_state ps6_traj_builder)

cs_install()
cs_export()
    

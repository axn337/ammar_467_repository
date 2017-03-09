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
cs_add_library(traj_builder src/traj_builder.cpp)   

# Executables
cs_add_executable(my_lidar_alarm src/my_lidar_alarm.cpp)
cs_add_executable(traj_builder_example_main src/traj_builder_example_main.cpp)
cs_add_executable(mobot_pub_des_state src/pub_des_state_main.cpp src/pub_des_state.cpp)
cs_add_executable(mobot_pub_des_state_startup src/pub_des_state_startup_main.cpp src/pub_des_state.cpp)
cs_add_executable(open_loop_controller src/open_loop_controller.cpp)
cs_add_executable(pub_des_state_path_client src/pub_des_state_path_client.cpp)
cs_add_executable(pub_des_state_path_client_3x3 src/pub_des_state_path_client_3mx3m_sqr.cpp)

#cs_add_executable(lidar_alarm2 src/lidar_alarm2.cpp)
# target_link_library(example my_lib)
target_link_libraries(traj_builder_example_main traj_builder)

cs_install()
cs_export()
    

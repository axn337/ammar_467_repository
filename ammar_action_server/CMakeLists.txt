cmake_minimum_required(VERSION 2.8.3)
project(ammar_action_server)

find_package(catkin_simple REQUIRED)
# example boost usage
find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

# Executables

cs_add_executable(ammar_action_server src/ammar_action_server.cpp)
cs_add_executable(ammar_action_client src/ammar_action_client.cpp)
cs_install()
cs_export()
    

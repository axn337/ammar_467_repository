# my_lidar_alarm

A submession for assignment 8 in EECS476. Refer to the write up.

# Running the program

roslaunch gazebo_ros empty_world.launch
roslaunch exmpl_models add_glennan_2nd_flr.launch
roslaunch mobot_urdf mobot_w_lidar.launch
roscd exmpl_models/glennan_2nd_flr, then:
rosrun map_server map_server glennan_2nd_flr_model_map.yaml
rosrun amcl amcl
rosrun mobot_drifty_odom mobot_drifty_odom
rosrun odom_tf odom_tf_demo


open loop:
rosrun ps8_axn337 ps8_open_loop_controller

lin_steering:
rosrun ps8_lin_steering ps8_lin_steering

Running Publisher ans client:
rosrun ps8_axn337 ps8_pub_des_state_path_client
rosrun ps8_axn337 ps8_pub_des_state

view triad_display bublished by OdomTF in RVIZ:
rosrun example_rviz_marker triad_display



# my_lidar_alarm

A submession for assignment six in EECS476. Refer to the write up.

# Running the program

roslaunch mobot_urdf mobot_in_pen.launch

rosrun assignment_6 ps6_lidar_alarm

rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=robot0/cmd_vel


rosrun assignment_6 ps6_pub_des_states 

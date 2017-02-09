# my_lidar_alarm

This code subscribes to topic `robot0/laser_0`, which is published by the Simple 2-D Robot simulator.
The signal interpretation in this example merely looks at a single ping--straight ahead.  If this
ping distance is less than some danger threshold, the lidar listener publishers a warning signal on
topic `lidar_alarm`.  The distance of the forward ping is also published, on topic `lidar_dist`.

I modified the code lidar_alarm found in https://github.com/wsnewman/cwru-ros-pkg-hydro-wsn/tree/master/catkin/src/cwru_376_student/lidar_alarm/src, such that the lidar scanner can detect a range of angles between -1.5 radians to 1.5 radians and measure the minimum safe distanse. If the minimum safe distant exceed the distant read for each reading, an alarm message will be pulished.

## Example usage
Start up the STDR simulator:
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
Start the lidar alarm node:
 `rosrun my_lidar_alarm my_lidar_alarm`
 Have the controller code monitor the `my_lidar_alarm` topic and do something intelligent with the information.

    

# my_stdr_control
a copy of stdr_control found in learning_ros/part2 directory from wsnewman git repository. The function is to control the STDR mobile robot with open-loop commands.

I edited my_stdr_open_loop_commander.cpp according to the first assignment. I also edited the CMake  and the package.xml files in order to recogonize the newly named nodes, publishers, and supscribers. 

## Example usage
`roslaunch stdr_launchers server_with_map_and_gui_plus_robot.launch`
to start the simulator.  Run a simple, open-loop command sequence with:
`rosrun my_stdr_control my_stdr_open_loop_commander`

    

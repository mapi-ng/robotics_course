#!/bin/sh
xterm -e  " roscore" &
sleep 3
xterm -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find maps)/worlds/world_proj.world" &
sleep 1
#xterm  -e  " roslaunch slam_gmapping turtlebot_world.launch " &
#sleep 5
xterm  -e  " rosrun rviz rviz" 

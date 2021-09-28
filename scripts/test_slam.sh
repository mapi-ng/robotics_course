#!/bin/sh
xterm -e  " roscore" &
sleep 3
xterm -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find maps)/worlds/world_proj.world" &
sleep 3
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 3
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch " &

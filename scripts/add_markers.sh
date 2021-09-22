#!/bin/sh
xterm -e  " roscore" &
sleep 3
xterm -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find maps)/worlds/world_proj.world" &
sleep 3
xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find maps)/maps/map.yaml " &
sleep 3
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 3
xterm  -e  " rosrun add_markers add_markers "

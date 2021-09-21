#!/bin/sh
xterm  -e  " source /opt/ros/kinetic/setup.bash; source devel/setup.bash; roscore" & 
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
#xterm  -e  " roslaunch slam_gmapping turtlebot_world.launch " &
#sleep 5
xterm  -e  " rosrun rviz rviz" 

#!/bin/sh
xterm -e  " roscore" &
sleep 3
xterm -e  " roslaunch my_robot world.launch" &
sleep 3
xterm  -e  " roslaunch my_robot amcl.launch map_file:=$(rospack find maps)/maps/map.yaml " &
sleep 3
xterm  -e  " rosrun pick_objects pick_objects " &
sleep 3
xterm  -e  " rosrun add_markers add_markers_odom_sub " &

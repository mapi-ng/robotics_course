# Repository for Udacity Robotics Software Engineer course

## Intro
Directory `scripts` contains shell scripts:
- test_slam.sh - manually test SLAM with TurtleBot
- test_navigation.sh - manually test navigation with TurtleBot
- pick_objects.sh - tests sending multiple goals, time based
- add_markers.sh - tests publishing markers, time based
- home_service.sh - script for the final project with custom robot

**Before executing, please navigate to the workspace directory and run following commands:**
```
catkin_make
source /opt/kinetic/setup.bash
source devel/setup.bash
```

## Packages
### my_robot
Package with robot description files, costmap and planner configuration.

#### Launching the world

`roslaunch my_robot world.launch`

#### Launching AMCL

`roslaunch my_robot amcl.launch`

### ball_chaser
Detects and follows a white object in the camera field of view.

#### Launching the ball chaser

`roslaunch ball_chaser ball_chaser.launch`

#### Nodes

##### drive_bot

Uses a service server for robot velocity requests and publishes `/cmd_vel` based on received data.

##### process_imagge

Subscribes to `/camera/rgb/image_raw` topic, searches for white object in the field of view and sends velocity requests to the `drive_bot` node in order to follow the object.

### pick_objects

#### Nodes

##### pick_objects
Sends two goals (pick up zone and drop off zone) for the robot to reach with 5 second intervals.

##### pick_objects_marker_sub
WORK IN PROGRESS - not required for the project.

Subscribes to visual marker topic and sends navigation goal based on the marker.

### add_markers
#### Nodes
##### add_markers
Publishes a marker at the pickup zone, after 5 seconds removes the marker. Then after another 5 seconds publishes a marker at the drop off zone.

##### add_markers_odom_sub
Publishes a marker at pick up zone. Then remvoes the marker once the robot arrives to the zone. Waits 5 seconds.
Once the robot reaches the drop off zone, publishes a marker in that location.

### maps
Package with map files and world description files.

### rvizConfig
Package with RViz configuration files - note **camelCase** is used only to meet project requirement

## External packages

Following external packages were added to the projects:
- pgm_map_creator
- slam_gmapping
- teleop_twist_keyboard
- turtlebot
- turtlebot_interactions
- turtlebot_simulator

# Repository for Udacity Robotics Software Engineer course

## Intro
Directory scripts contains shell scripts:
- test_slam.sh - used to manually test SLAM with TurtleBot
- test_navigation.sh - used to manually test navigation with TurtleBot
- pick_objects.sh - tests sending multiple goals, time based
- add_markers.sh - tests publishing markers, time based
- home_service.sh - script for the final project

**Before executing, please navigate to the workspace directory and run following commands:**
```
source /opt/kinetic/setup.bash
source devel/setup.bash
```

Repository comprises following packages:

- my_robot - package with robot and world description files,
- ball_chaser - package responsible for detection and following of a white object in the camera field of view,
- pick_objects - package contains pick_objects node which sends two goals for the robot to reach with 5 s intervals,
- add_markers - package contains add_markers node which publishes pick up and drop off markers with time intervals, as well as add_markers_odom_sub which publishes and removes markers based on robot position in the map
- maps - package with map files and world files
- rvizConfig - package with RViz configuration files - note **camelCase** is used only to meet project requirement

Following external packages were added to the projects:
- pgm_map_creator
- slam_gmapping
- teleop_twist_keyboard
- turtlebot
- turtlebot_interactions
- turtlebot_simulator

## my_robot

### Launching the world

`roslaunch my_robot world.launch`

## ball_chaser

### Launching the nodes

`roslaunch ball_chaser ball_chaser.launch`

### Nodes

#### drive_bot

Uses a service server for robot velocity requests and publishes `/cmd_vel` based on received data.

#### process_imagge

Subscribes to `/camera/rgb/image_raw` topic, searches for white object in the field of view and sends velocity requests to the `drive_bot` node in order to follow the object.
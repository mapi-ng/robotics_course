# Repository for Udacity Robotics Software Engineer course

Repository comprises following packages:

- my_robot - package with robot and world description files,
- ball_chaser - package responsible for detection and following of a white object in the camera field of view.

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
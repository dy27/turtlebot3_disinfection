# turtlebot3_disinfection

## Overview
This package implements five ROS nodes which demonstrate functionality for a Turtlebot3 robot to perform simulated disinfection of workspaces within an environment
and monitor social distancing between people. This functionality is developed on a small scale, where people and workspaces are represented by April tags. The
functionality provided in this package could be scaled for real world operation by modifying the April tag scanning module to implement detection of real people and
workspaces.

## ROS Nodes

### motion_planner
Controls the motion of the robot depending on the current robot state. The three main states of the robot are moving/wall-following, stopped/disinfecting and 
rotating/scanning. 

### tag_scanner
Monitors the camera image data to detect April tags reprenting people or workspaces.

### scan_database
Receives detection data from the tag_scanner node and stores them in a database for contact tracing. Implements query commands for retrieval of information.

### social_distance_monitor
Receives detection data from the tag_scanner node and monitors for violations of social distancing. Publishes alerts for each violation.

### map_transform
Computes the pose of the robot in the global map frame by using odometry data.


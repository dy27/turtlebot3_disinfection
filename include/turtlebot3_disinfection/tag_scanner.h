/**
 * tag_scanner.h
 *
 * Node for scanning April tags corresponding to people and workspaces. When a workspace is detected, a scan is
 * performed to detect people working at the workspace.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#ifndef TAG_SCANNER_
#define TAG_SCANNER_

#include "ros/ros.h"
#include "turtlebot3_disinfection/motion_planner.h" // Access to the RobotState enum
#include "turtlebot3_disinfection/Scan.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>
#include <cmath>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>


class TagScanner
{
    public:
        // Constructs a class instance and initialises publishers and subscribers.
        TagScanner(ros::NodeHandle* nh);

        // Publishes to robot_state to request a change in behaviour in the motion_planner node.
        void publishRobotState(int state) const;

        // Callback function for April tag detections.
        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg);

        // Adds a tag detection to the array of detections stored for the current scan.
        void addDetection(int tag_id, const apriltag_ros::AprilTagDetection& apriltag_msg);

        // Callback function which converts all the detections found during the scan into a Scan msg to send to the
        // database node.
        void scanCompleteCallback(const std_msgs::Empty& msg);

        // Callback function which  updates the current_map_pose value stored in this class.
        void mapPoseCallback(const geometry_msgs::PoseStamped& msg);

    private:
        // Integer value representing the robot state.
        int robot_state_;

        // Map from each detected April tag ID to its pose for a particular scan
        std::unordered_map<int,geometry_msgs::PoseStamped> detections_;

        // The tag ID of the last workspace which was scanned.
        int last_workspace_scanned_id;

        // The time at which the last workspace was scanned.
        ros::Time last_workspace_scanned_time;

        // The current pose of the robot in the map frame.
        geometry_msgs::PoseStamped current_map_pose;

        // Publisher to robot state.
        const ros::Publisher pub_robot_state_;

        // Publishes Scan messages to the database.
        const ros::Publisher pub_tag_scan_;

        // Subscribes to when the robot has completed a full rotation for a scan.
        const ros::Subscriber sub_rotation_complete_;

        // Subscribes to April tag detections for each camera image frame.
        const ros::Subscriber sub_apriltag_;

        // Subscribes to the robot pose published by map_transform.
        const ros::Subscriber sub_map_pose_;
};

#endif

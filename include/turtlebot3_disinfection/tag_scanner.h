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
#include <utility>      // std::pair
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <apriltag_ros/AprilTagDetectionArray.h>


class TagScanner
{
    public:

        TagScanner(ros::NodeHandle* nh);

        void publishRobotState(int state);

        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg);

        void addDetection(int tag_id, const apriltag_ros::AprilTagDetection& apriltag_msg);

        void scanCompleteCallback(const std_msgs::Empty& msg);

        void mapPoseCallback(const geometry_msgs::PoseStamped& msg);

    private:
        int robot_state_;

        // int scan_count_;

        // Workspace* detected_workspace;
        // // std::vector<Person> detected_people;
        std::unordered_map<int,geometry_msgs::PoseStamped> detections_;

        geometry_msgs::PoseStamped current_map_pose;

        const ros::Publisher pub_robot_state_;

        const ros::Publisher pub_tag_scan_;

        const ros::Subscriber sub_rotation_complete_;

        const ros::Subscriber sub_apriltag_;

        const ros::Subscriber sub_map_pose_;
};

#endif

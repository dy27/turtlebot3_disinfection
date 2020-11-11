#ifndef DISINFECTION_CONTROL_
#define DISINFECTION_CONTROL_

#include "ros/ros.h"
// #include "disinfection_database.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>
#include <std_msgs/Bool.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <utility>      // std::pair


class DisinfectionControl
{
    public:
        DisinfectionControl(ros::NodeHandle* nh);

        void publishRobotState(int state);

        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg);

        void addDetection(int tag_id, const apriltag_ros::AprilTagDetection& apriltag_msg);

        void scanCompleteCallback(const std_msgs::Bool& msg);

        void mapPoseCallback(const geometry_msgs::PoseStamped& msg);

        void loop();

    private:
        int robot_state_;

        int scan_count_;

        // Workspace* detected_workspace;
        // // std::vector<Person> detected_people;
        std::unordered_map<int,std::vector<float>> detections_;

        geometry_msgs::PoseStamped current_map_pose;

        const ros::Publisher pub_robot_state_;

        const ros::Subscriber sub_scan_complete_;

        const ros::Subscriber sub_apriltag_;

        const ros::Subscriber sub_map_pose_;
};

#endif

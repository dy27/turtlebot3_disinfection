#ifndef SOCIAL_DISTANCE_MONITOR_
#define SOCIAL_DISTANCE_MONITOR_

#include "ros/ros.h"
// #include "disinfection_database.h"
// #include "turtlebot3_disinfection/Scan.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

class SocialDistanceMonitor
{
    public:
        SocialDistanceMonitor(ros::NodeHandle* nh, float separation_distance);

        float calculateDistance(const geometry_msgs::Pose& msg1, const geometry_msgs::Pose& msg2);

        // void tagScanCallback(const turtlebot3_disinfection::Scan& msg);

        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg);

    private:

        const float sep_distance_;

        const ros::Subscriber sub_apriltag_;
};

#endif

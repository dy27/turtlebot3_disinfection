/**
 * social_distance_monitor.h
 *
 * This node subscribes to all April tag detections and monitors distances between people to enforce social distancing.
 * An alert is issued when a pair of people violate the distancing requirements.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#ifndef SOCIAL_DISTANCE_MONITOR_
#define SOCIAL_DISTANCE_MONITOR_

#include "ros/ros.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>


class SocialDistanceMonitor
{
    public:
        // Class constructor which initialises subscribers and sets the separation distance.
        SocialDistanceMonitor(ros::NodeHandle* nh, float separation_distance);

        // Calculates the Euclidean distance between two poses in 3D space.
        float calculateDistance(const geometry_msgs::Pose& msg1, const geometry_msgs::Pose& msg2) const;

        // Callback function which prints the distance between people.
        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg) const;

    private:
        // The distance threshold for social distancing alerts.
        const float sep_distance_;

        // Subscriber to April tag detections.
        const ros::Subscriber sub_apriltag_;
};

#endif

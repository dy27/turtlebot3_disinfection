#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include "ros/ros.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


class MotionPlanner
{
    public:
        MotionPlanner(ros::NodeHandle* nh);

        void publishVelocity(const float lin_vel, const float ang_vel);

        float getRange(const sensor_msgs::LaserScan& msg, int index);

        float getMinRange(const sensor_msgs::LaserScan& msg, int start_angle, int end_angle);

        float median(std::vector<float>& distances);

        float mapToRange(float value, const std::vector<float>& range, const std::vector<float>& new_range,
            bool saturate);

        void laserCallback(const sensor_msgs::LaserScan& msg);

    private:

        int mode_;

        const ros::Publisher pub_vel_;

        const ros::Subscriber sub_laser_;
};

#endif

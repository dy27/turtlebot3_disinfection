#ifndef MOTIONPLANNER_H_
#define MOTIONPLANNER_H_

#include "ros/ros.h"

#include <iostream>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class MotionPlanner
{
    public:
        MotionPlanner(ros::NodeHandle* nh);

        void publishVelocity(const std::vector<float> lin_vel, const std::vector<float> ang_vel);

        void laserCallback(const sensor_msgs::LaserScan& msg);

    private:
        ros::Publisher pub_vel_;

        ros::Subscriber sub_laser_;

};

#endif

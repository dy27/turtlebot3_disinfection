#ifndef SPRAY_CONTROL_H_
#define SPRAY_CONTROL_H_

#include "ros/ros.h"

#include <iostream>
// #include <string>
// #include <vector>
// #include <sensor_msgs/LaserScan.h>
// #include <geometry_msgs/Twist.h>

class SprayControl
{
    public:
        SprayControl(ros::NodeHandle* nh);

        void publishVelocity(const std::vector<float> lin_vel, const std::vector<float> ang_vel);

        void laserCallback(const sensor_msgs::LaserScan& msg);

    private:
        bool spray_on_;

        ros::Publisher pub_vel_;

        ros::Subscriber sub_laser_;

};

#endif

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

        float getMinRange(const sensor_msgs::LaserScan& msg, const std::vector<int>& angle_range);

        float median(std::vector<float>& distances);

        float mapToRange(float value, const std::vector<float>& range, const std::vector<float>& new_range,
            bool saturate);

        void laserCallback(const sensor_msgs::LaserScan& msg);

        template <class T>
        T getParam(ros::NodeHandle* nh, std::string param_name)
        {
            T param;
            if(!nh->getParam(param_name, param)) {
                ROS_ERROR("Parameter failed to load");
            }
            return param;
        }

    private:

        const float WALL_DIST;
        const float FRONT_TURN_DIST;
        const float WALL_MAX_TRACK_DIST;
        const float MAX_LIN_VEL;
        const float MAX_ANG_VEL;
        const float MAX_RANGE;

        const std::vector<int> FRONT_RANGE;
        const std::vector<int> LEFT_RANGE;
        const std::vector<int> LEFT_FRONT_RANGE;
        const std::vector<int> LEFT_BACK_RANGE;

        const std::vector<float> ANGLE_ERROR_RANGE;
        const std::vector<float> DIST_ERROR_RANGE;
        const std::vector<float> ANG_VEL_RANGE;

        const ros::Publisher pub_vel_;

        const ros::Subscriber sub_laser_;
};

#endif

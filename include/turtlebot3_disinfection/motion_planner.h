#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include "ros/ros.h"

#include <iostream>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf2_ros/transform_listener.h>

class MotionPlanner
{
    public:
        MotionPlanner(ros::NodeHandle* nh);

        // void publishVelocity(const std::vector<float> lin_vel, const std::vector<float> ang_vel);

        // void laserCallback(const sensor_msgs::LaserScan& msg);

    private:
        std::vector<std::vector<int>> waypoints_;

        // ros::Publisher pub_vel_;

        ros::Publisher pub_waypoint_;

        ros::Subscriber sub_map_;

        // ros::Subscriber sub_laser_;



};

#endif

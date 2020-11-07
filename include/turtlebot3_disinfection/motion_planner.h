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

#define POSITION_THRESHOLD 0.05f

class MotionPlanner
{
    public:
        MotionPlanner(ros::NodeHandle* nh);

        void publishWaypoint(const std::vector<float> position, const std::vector<float> orientation);

        // void publishVelocity(const std::vector<float> lin_vel, const std::vector<float> ang_vel);

        // void laserCallback(const sensor_msgs::LaserScan& msg);

        void mapCallback(const nav_msgs::OccupancyGrid& msg);

        void mapPoseCallback(const geometry_msgs::PoseStamped& msg);

    private:
        geometry_msgs::PoseStamped current_pose_;

        geometry_msgs::PoseStamped waypoint_;

        std::vector<std::vector<int>> waypoints_;

        // ros::Publisher pub_vel_;

        ros::Publisher pub_waypoint_;

        ros::Subscriber sub_map_;

        ros::Subscriber sub_map_pose_;

        // ros::Subscriber sub_laser_;



};

#endif

#ifndef MAP_TRANFORM_H_
#define MAP_TRANFORM_H_

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

class MapTransform
{
    public:
        MapTransform(ros::NodeHandle* nh);

        void publishPose(const std::vector<float> position, const std::vector<float> orientation);

        // void publishVelocity(const std::vector<float> lin_vel, const std::vector<float> ang_vel);

        // void laserCallback(const sensor_msgs::LaserScan& msg);

    private:
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // ros::Publisher pub_vel_;

        ros::Publisher pub_pose_;

        // ros::Subscriber sub_map_;

        // ros::Subscriber sub_laser_;



};

#endif

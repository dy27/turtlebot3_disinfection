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

        void publishPose();

    private:
        tf2_ros::Buffer tf_buffer_;

        tf2_ros::TransformListener tf_listener_;

        ros::Publisher pub_pose_;
};

#endif
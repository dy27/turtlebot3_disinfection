/**
 * map_transform.cpp
 *
 * This node continuosly publishes the PoseStamped message which contains the pose of the robot in the global map frame.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#ifndef MAP_TRANFORM_H_
#define MAP_TRANFORM_H_

#include "ros/ros.h"

#include <iostream>
#include <string>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>


class MapTransform
{
    public:
        // Class constructor which initialises the pose publisher.
        MapTransform(ros::NodeHandle* nh, const tf2_ros::Buffer& tf_buffer);

        // Computes the transform from map to base_link and publishes it on the /map_pose topic.
        void publishPose() const;

    private:
        // Reference to a Buffer object from the tf2_ros package to store transforms.
        const tf2_ros::Buffer& tf_buffer_;

        // The publisher used to publish the computed poses.
        ros::Publisher pub_pose_;
};

#endif

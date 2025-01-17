/**
 * motion_planner.h
 *
 * This node controls all navigation and movement of the robot by publishing to the /cmd_vel topic.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#ifndef MOTION_PLANNER_H_
#define MOTION_PLANNER_H_

#include "ros/ros.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>


class MotionPlanner
{
    public:
        // Enumeration of robot state values.
        enum RobotState {
            WALL_FOLLOWING,
            STOPPED,
            TAG_SCANNING
        };

        // Class constructor which initialises publishers and subscribers, and loads parameters.
        MotionPlanner(ros::NodeHandle* nh);

        // Publishes to the /cmd_vel topic to make the robot move at the specified linear and angular velocities.
        void publishVelocity(float lin_vel, float ang_vel) const;

        // Publishes to the /rotation_complete topic to signal that a full rotation has been performed by the robot.
        void publishScanComplete() const;

        // Returns the distance at a certain index in the array of measurements from the laser.
        float getRange(const sensor_msgs::LaserScan& msg, int index, int n_measurements) const;

        // Returns the median of a vector of float values.
        float median(std::vector<float>& distances) const;

        // Returns the closest distance measurement from the LIDAR within a specified angle range.
        float getMinRange(const sensor_msgs::LaserScan& msg, const std::vector<int>& angle_range,
            int* min_index_result) const;

        // Given a value, an interval, and a new interval, computes the new value which divides the new interval in the
        // same ratio that the given value divides the given interval.
        float mapToRange(float value, const std::vector<float>& range, const std::vector<float>& new_range,
            bool saturate) const;

        // Controls the robot motion to make it follow a nearby left wall.
        void robotFollowWall(const sensor_msgs::LaserScan& msg) const;

        // Controls the robot motion to make it perform a scan of its surroundings by rotating once.
        void robotRotate(const sensor_msgs::LaserScan& msg);

        // Callback function for the LaserScan msg which contains the LIDAR measurements.
        void laserCallback(const sensor_msgs::LaserScan& msg);

        //  Callback function to update the state of the robot.
        void robotStateCallback(const std_msgs::Int8& msg);

        // Function to load ROS parameters from the parameter server
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
        // Integer representing the state of the robot, refer to the RobotState enum
        int robot_state_;

        // Integer representing the scan progress as the robot is doing a full revolution to scan its surroundings
        int scan_progress_;

        // Robot velocity publisher
        const ros::Publisher pub_vel_;

        // LIDAR scan subscriber
        const ros::Subscriber sub_laser_;

        // Robot state subscriber
        const ros::Subscriber sub_robot_state_;

        // Rotation complete publisher
        const ros::Publisher pub_rotation_complete_;

        /* Wall-following Algorithm Parameters */
        const float WALL_DIST; // Distance to maintain from the left wall
        const float FRONT_TURN_DIST; // Distance to front object before a right turn is performed
        const float WALL_MAX_TRACK_DIST; // The maximum distance on the left where a wall will still be tracked
        const float MAX_LIN_VEL; // Maximum linear velocity
        const float MAX_ANG_VEL; // Maximum angular velocity
        const float MAX_RANGE; // Maximum LIDAR range
        const std::vector<int> FRONT_RANGE; // Front angle range for detecting objects in front
        const std::vector<int> LEFT_RANGE; // Left angle range for wall tracking
        const std::vector<int> LEFT_FRONT_RANGE; // Left front angle range for wall slope estimation
        const std::vector<int> LEFT_BACK_RANGE; // Left back angle range for wall slope estimation
        const std::vector<float> DIST_ERROR_RANGE; // The dist_error range to map to the range of angular velocities
        const std::vector<float> ANGLE_ERROR_RANGE; // The angle_error range to map to the range of angular velocities
        const std::vector<float> ANG_VEL_RANGE; // The angular velocity range which the error ranges map to


};

#endif

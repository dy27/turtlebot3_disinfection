/**
 * ... text ...
 */

#include "turtlebot3_disinfection/motion_planner.h"


MotionPlanner::MotionPlanner(ros::NodeHandle* nh)
    : robot_state_(0)
    , pub_vel_(nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10))
    , sub_laser_(nh->subscribe("/scan", 1000, &MotionPlanner::laserCallback, this))
    , sub_robot_state_(nh->subscribe("/robot_state", 1000, &MotionPlanner::robotStateCallback, this))
    , sub_scan_complete_(nh->subscribe("/scan_complete", 1000, &MotionPlanner::scanCompleteCallback, this))
    , WALL_DIST(getParam<float>(nh, "wall_distance"))
    , FRONT_TURN_DIST(getParam<float>(nh, "front_turn_distance"))
    , WALL_MAX_TRACK_DIST(getParam<float>(nh, "wall_track_distance"))
    , MAX_LIN_VEL(getParam<float>(nh, "max_linear_velocity"))
    , MAX_ANG_VEL(getParam<float>(nh, "max_angular_velocity"))
    , MAX_RANGE(getParam<float>(nh, "max_scan_range"))
    , FRONT_RANGE(getParam<std::vector<int>>(nh, "front_angle_range"))
    , LEFT_RANGE(getParam<std::vector<int>>(nh, "left_angle_range"))
    , LEFT_FRONT_RANGE(getParam<std::vector<int>>(nh, "left_front_angle_range"))
    , LEFT_BACK_RANGE(getParam<std::vector<int>>(nh, "left_back_angle_range"))
    , ANGLE_ERROR_RANGE(getParam<std::vector<float>>(nh, "angle_error_range"))
    , DIST_ERROR_RANGE(getParam<std::vector<float>>(nh, "distance_error_range"))
    , ANG_VEL_RANGE({-MAX_ANG_VEL, MAX_ANG_VEL})
{
}


void MotionPlanner::publishVelocity(const float lin_vel, const float ang_vel)
{
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = lin_vel;
    vel_msg.linear.y = 0.0f;
    vel_msg.linear.z = 0.0f;
    vel_msg.angular.x = 0.0f;
    vel_msg.angular.y = 0.0f;
    vel_msg.angular.z = ang_vel;

    pub_vel_.publish(vel_msg);
}


float MotionPlanner::getRange(const sensor_msgs::LaserScan& msg, int index)
{
    std::vector<float> ranges;
    ranges.reserve(5);
    float range;
    for (int i=0; i<5; i++)
    {
        range = msg.ranges[index-2];
        if (range < 0.001 || range > msg.range_max)
        {
            range = MAX_RANGE;
        }
        ranges[i] = range;
    }

    return median(ranges);
}


float MotionPlanner::median(std::vector<float>& distances)
{
    std::sort(distances.begin(), distances.end());
    int index = distances.size() / 2;
    if (distances.size() % 2 == 0)
    {
        return (distances[index] + distances[index+1]) / 2;
    }
    else
    {
        return distances[index];
    }
}


float MotionPlanner::mapToRange(float value, const std::vector<float>& range, const std::vector<float>& new_range,
    bool saturate=true)
{
    float mapped_value;

    if (value < range[0] && saturate)
    {
        mapped_value = new_range[0];
    }
    else if (value > range[1] && saturate)
    {
        mapped_value = new_range[1];
    }
    else
    {
        float percent = (value - range[0]) / (range[1] - range[0]);

        mapped_value = new_range[0] + percent * (new_range[1] - new_range[0]);
    }

    return mapped_value;
}


float MotionPlanner::getMinRange(const sensor_msgs::LaserScan& msg, const std::vector<int>& angle_range,
    int* min_index_result=NULL)
{
    int start_angle = angle_range[0];
    int end_angle = angle_range[1];

    while (start_angle > end_angle)
    {
        end_angle += 360;
    }

    float min_range = MAX_RANGE + 1;
    int min_index = -1;
    float range;
    for (int i=start_angle; i<=end_angle; i++)
    {
        range = getRange(msg, i % 360);
        if (range < min_range)
        {
            min_range = range;
            min_index = i;

        }
    }

    if (min_index_result != NULL)
    {
        *min_index_result = min_index;
    }

    return min_range;
}


void MotionPlanner::laserCallback(const sensor_msgs::LaserScan& msg)
{
    ROS_INFO("robot_state: %d", robot_state_);

    if (robot_state_ == 1)
    {
        publishVelocity(0, 0);
        return;
    }
    else if (robot_state_ == 2)
    {
        // Find the angle to the wall closest to the robot
        int min_index;
        const std::vector<int> full_range = {0, 359};
        getMinRange(msg, full_range, &min_index);

        // If scan is complete and the closest wall is in LEFT_RANGE
        const int right_range_start = 250;
        const int right_range_end = 290;
        const int left_range_start = 70;
        const int left_range_end = 110;

        if (scan_state_ == 0 && min_index >= right_range_start && min_index <= right_range_end)
        {
            ROS_INFO("Half turn complete");
            scan_state_ = 1;
        }
        else if (scan_state_ == 1 && min_index >= left_range_start && min_index <= left_range_end)
        {
            ROS_INFO("Full turn complete");

            // Change robot state back to wall following
            robot_state_ = 0;
            scan_complete_ = false;
        }
        else
        {
            // Keep spinning clockwise
            publishVelocity(0, -MAX_ANG_VEL/3);
        }
        return;
    }

    float lin_vel;
    float ang_vel;

    float left_min_range = getMinRange(msg, LEFT_RANGE);
    float dist_error = left_min_range - WALL_DIST;

    ROS_INFO("range: %.3f\tdiff: %.3f", left_min_range, dist_error);

    float angle_error = getMinRange(msg, LEFT_FRONT_RANGE) - getMinRange(msg, LEFT_BACK_RANGE);

    // if (getRange(msg, LEFT_BACK) > 0.3f)
    // {
    //     angle_error = 0.0f;
    // }

    // ROS_INFO("leftfront: %.3f\tleftback: %.3f\terr: %.3f", getRange(msg, LEFT_FRONT), getRange(msg, LEFT_BACK), angle_error);

    float dist_error_ang_vel = mapToRange(dist_error, DIST_ERROR_RANGE, ANG_VEL_RANGE);
    ROS_INFO("dist error %.3f mapped to dist_ang_vel %.3f", dist_error, dist_error_ang_vel);

    float angle_error_ang_vel = mapToRange(angle_error, ANGLE_ERROR_RANGE, ANG_VEL_RANGE);
    ROS_INFO("ang error %.3f mapped to ang_ang_vel %.3f", angle_error, angle_error_ang_vel);

    ang_vel = dist_error_ang_vel + angle_error_ang_vel;
    if (ang_vel > MAX_ANG_VEL)
    {
        ang_vel = MAX_ANG_VEL;
    }
    else if (ang_vel < -MAX_ANG_VEL)
    {
        ang_vel = -MAX_ANG_VEL;
    }

    // TODO: move this up somewhere
    if (left_min_range > WALL_MAX_TRACK_DIST)
    {
        ang_vel = 0;
    }

    lin_vel = MAX_LIN_VEL;

    float min_range_front = getMinRange(msg, FRONT_RANGE);
    ROS_INFO("min_range_front: %.3f", min_range_front);
    if (min_range_front < FRONT_TURN_DIST)
    {
        lin_vel = 0;
        ang_vel = -MAX_ANG_VEL;
    }

    publishVelocity(lin_vel, ang_vel);
}

void MotionPlanner::robotStateCallback(const std_msgs::Int8& msg)
{
    robot_state_ = msg.data;
    if (robot_state_ == 2)
    {
        scan_state_ = 0;
    }
}

void MotionPlanner::scanCompleteCallback(const std_msgs::Bool& msg)
{
    scan_complete_ = true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    MotionPlanner motion_planner = MotionPlanner(&nh);

    ros::spin();

    return 0;
}

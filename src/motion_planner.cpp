/**
 * ... text ...
 */

#include "turtlebot3_disinfection/motion_planner.h"

#define WALL_DIST 0.2f
#define MAX_LIN_VEL 0.22f
#define MAX_ANG_VEL 2.84f
#define P_LINEAR 1.0f
#define P_ANGULAR 1.0f
#define D_LINEAR 1.0f
#define D_ANGULAR 1.0f

#define FRONT 0
#define LEFT 90
#define LEFT_FRONT 85
#define LEFT_BACK 95


// const int SCAN_RANGES = {0, }

MotionPlanner::MotionPlanner(ros::NodeHandle* nh)
    : mode_(0)
    , pub_vel_(nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10))
    , sub_laser_(nh->subscribe("/scan", 1000, &MotionPlanner::laserCallback, this))
{
    // pub_vel_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // sub_laser_ = nh->subscribe("/scan", 1000, &MotionPlanner::laserCallback, this);
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

float getRange(const sensor_msgs::LaserScan& msg, int index)
{
    return std::min(msg.ranges[index], msg.range_max);
}

void MotionPlanner::laserCallback(const sensor_msgs::LaserScan& msg)
{
    // positive -> turn left
    float p_error = (msg.ranges[LEFT] - WALL_DIST) / msg.ranges[LEFT];

    // positive -> turn left
    float d_error = (msg.ranges[LEFT_FRONT] - msg.ranges[LEFT_BACK]) / (msg.ranges[LEFT_FRONT] + msg.ranges[LEFT_BACK]);

    float lin_vel = std::max(D_LINEAR / d_error, MAX_LIN_VEL);

    float ang_vel = std::max(P_ANGULAR * p_error + D_ANGULAR * d_error, MAX_ANG_VEL);

    publishVelocity(lin_vel, ang_vel);

    // float front_range = msg.ranges[0];
    // float left_range = msg.ranges[90];
    //
    // if (front_range > WALL_DIST && left_range > WALL_DIST)
    // {
    //     publishVelocity(0.6 * MAX_LIN_VEL, -0.3 * MAX_ANG_VEL);
    // }
    // else if (front_range < WALL_DIST && left_range > WALL_DIST)
    // {
    //     publishVelocity(0.0f, 0.2 * MAX_ANG_VEL);
    // }
    // else if (front_range > WALL_DIST && left_range < WALL_DIST)
    // {
    //     publishVelocity(0.6 * MAX_LIN_VEL, 0.0f);
    // }
    // else
    // {
    //     publishVelocity(0.0f, 0.6 * MAX_ANG_VEL);
    // }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    MotionPlanner motion_planner = MotionPlanner(&nh);

    // ros::Rate loop_rate(10);

    // initialise 0-vectors
    // std::vector<float> position(3, 0);
    // std::vector<float> orientation(4, 0);
    // position[0] = 1;
    // orientation[3] = 1;
    //
    // motion_planner.publishWaypoint(position, orientation);

    ros::spin();

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //
    //
    //     // motion_planner.updateWaypoint();
    //
    //
    //     loop_rate.sleep();
    // }

    return 0;
}

/**
 * ... text ...
 */

#include "turtlebot3_disinfection/motion_planner.h"


MotionPlanner::MotionPlanner(ros::NodeHandle* nh)
{
    pub_waypoint_ = nh->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // pub_vel_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // sub_laser_ = nh->subscribe("/scan", 1000, &MotionPlanner::laserCallback, this);
    sub_map_ = nh->subscribe("/map", 10, &MotionPlanner::mapCallback, this);

    sub_map_pose_ = nh->subscribe("/map_pose", 10, &MotionPlanner::mapPoseCallback, this);
}


void MotionPlanner::updateWaypoint()
{
    // Check if waypoint has been reached

    // Compare the current pose with the destination pose
    // TODO: Should change this to check Euclidean distance
    if (abs(current_pose_.pose.position.x - waypoint_.pose.position.x) < POSITION_THRESHOLD &&
        abs(current_pose_.pose.position.y - waypoint_.pose.position.y) < POSITION_THRESHOLD)
    {
        // waypoint_reached_ = true;
        waypoint_.pose.position.x += 1;
    }
}


void MotionPlanner::publishWaypoint(const std::vector<float> position, const std::vector<float> orientation)
{
    geometry_msgs::PoseStamped msg;

    msg.header.frame_id = "map";

    msg.pose.position.x = position[0];
    msg.pose.position.y = position[1];
    msg.pose.position.z = position[2];

    msg.pose.orientation.x = orientation[0];
    msg.pose.orientation.y = orientation[1];
    msg.pose.orientation.z = orientation[2];
    msg.pose.orientation.w = orientation[3];

    pub_waypoint_.publish(msg);
}


void MotionPlanner::mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    // int8** map = msg.data;
    map_ = msg;
}


void MotionPlanner::mapPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    current_pose_ = msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    MotionPlanner motion_planner = MotionPlanner(&nh);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        // initialise 0-vectors
        std::vector<float> position(3, 0);
        std::vector<float> orientation(4, 0);

        position[0] = 2;
        orientation[3] = 1;

        motion_planner.publishWaypoint(position, orientation);

        loop_rate.sleep();
    }

    return 0;
}

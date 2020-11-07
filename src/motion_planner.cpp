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


// void MotionPlanner::updateWaypoint()
// {
//
// }


// void MotionPlanner::publishVelocity(const std::vector<float> lin_vel, const std::vector<float> ang_vel)
// {
//     geometry_msgs::Twist msg;
//
//     msg.linear.x = lin_vel[0];
//     msg.linear.y = lin_vel[1];
//     msg.linear.z = lin_vel[2];
//
//     msg.angular.x = ang_vel[0];
//     msg.angular.y = ang_vel[1];
//     msg.angular.z = ang_vel[2];
//
//     pub_vel_.publish(msg);
// }

void MotionPlanner::publishWaypoint(const std::vector<float> position, const std::vector<float> orientation)
{
    geometry_msgs::PoseStamped msg;

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
}


void MotionPlanner::mapPoseCallback(const geometry_msgs::PoseStamped& msg)
{

    // // Compare the current pose with the destination pose
    // if (abs(msg.pose.position.x - waypoint_.pose.position.x) < POSITION_THRESHOLD &&
    //     abs(msg.pose.position.y - waypoint_.pose.position.y) < POSITION_THRESHOLD &&
    //     abs(msg.pose.position.z - waypoint_.pose.position.z) < POSITION_THRESHOLD)
    // {
    //     waypoint_reached_ = true;
    // }
    current_pose_ = msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner"); // Register the node on ROS

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    MotionPlanner motion_planner = MotionPlanner(&nh);

    ROS_INFO("Node started");

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        // initialise 0-vectors
        std::vector<float> position( 3, 0 );
        std::vector<float> orientation( 4, 0 );
        // lin_vel[0] = 1; // go forwards

        motion_planner.publishWaypoint(position, orientation);

        loop_rate.sleep();
    }

    ROS_INFO("Node finished");

    return 0;
}

// int main(int argc, char** argv)
// {
//     // ROS set-ups:
//     ros::init(argc, argv, "exampleRosClass"); //node name
//
//     ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
//
//     ROS_INFO("main: instantiating an object of type ExampleRosClass");
//     ExampleRosClass exampleRosClass(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
//
//     ROS_INFO("main: going into spin; let the callbacks do all the work");
//     ros::spin();
//     return 0;
// }

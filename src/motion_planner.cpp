/**
 * ... text ...
 */

#include "turtlebot3_disinfection/motion_planner.h"


MotionPlanner::MotionPlanner(ros::NodeHandle* nh)
{
    pub_waypoint_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // pub_vel_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // sub_laser_ = nh->subscribe("/scan", 1000, &MotionPlanner::laserCallback, this);
    sub_map_ = nh->subscribe<nav_msgs::OccupancyGrid>("/map", 10);
}

void MotionPlanner::updatePath()
{

}




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


void MotionPlanner::laserCallback(const sensor_msgs::LaserScan& msg)
{
    //message members: http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html

    //example for how to access members
    float min_angle = msg.angle_min;
}


void MotionPlanner::mapCallback(const nav_msgs::OccupancyGrid& msg)
{
    // int8** map = msg.data;
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
        std::vector<float> lin_vel( 3, 0 );
        std::vector<float> ang_vel( 3, 0 );
        lin_vel[0] = 1; // go forwards

        motion_planner.publishVelocity(lin_vel, ang_vel);

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

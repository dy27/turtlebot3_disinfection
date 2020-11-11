/**
 * ... text ...
 */

#include "turtlebot3_disinfection/disinfection_control.h"


DisinfectionControl::DisinfectionControl(ros::NodeHandle* nh)
    : pub_robot_state_(nh->advertise<std_msgs::Int8>("/robot_state", 10))
    , pub_scan_complete_(nh->advertise<std_msgs::Bool>("/scan_complete", 10))
    , sub_apriltag_(nh->subscribe("/tag_detections", 1000, &DisinfectionControl::tagDetectionCallback, this))
{
}

void DisinfectionControl::publishRobotState(int state)
{
    std_msgs::Int8 msg;
    msg.data = state;
    pub_robot_state_.publish(msg);
}

void DisinfectionControl::publishScanComplete(bool scan_complete)
{
    std_msgs::Bool msg;
    msg.data = scan_complete;
    pub_scan_complete_.publish(msg);
}

void DisinfectionControl::tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
    // Iterate through all detected tags
    for (const auto& detection : msg.detections)
    {
        // If a workspace is detected and the robot is current in wall following mode
        // if (detection.id[0] >= 0 && robot_state_ == 0)
        if (detection.id[0] >= 0)
        {
            // Set the robot state to stopped
            // robot_state_ = 2;
            publishRobotState(2);
        }

        // if (detection.id < 100 && robot_state_ == 2)
        // {
        //     Person person;
        //     p.id = detection.tag_id;
        //
        //     if (detected_people.find(detection.id) == detected_people.end())
        //     {
        //         detected_people.insert(detection.tag_id, person);
        //     }
        //     else
        //     {
        //
        //     }
        // }
    }
}

void DisinfectionControl::loop()
{
    while (ros::ok())
    {
        ros::spinOnce();

        // if (robot_state_ == 1)
        // {
        //     robot_state = 2;
        //     publishRobotState(2);
        // }
        // else if (robot_state == 2)
        // {
        //
        // }

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "disinfection_control");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    DisinfectionControl disinfection_control = DisinfectionControl(&nh);

    disinfection_control.loop();

    // ros::spin();

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //
    //     if ()
    // }

    return 0;
}

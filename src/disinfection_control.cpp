/**
 * ... text ...
 */

#include "turtlebot3_disinfection/disinfection_control.h"


DisinfectionControl::DisinfectionControl(ros::NodeHandle* nh)
    : pub_robot_state_(nh->advertise<std_msgs::Int8>("/robot_state", 10))
    , sub_scan_complete_(nh->subscribe("/scan_complete", 1000, &DisinfectionControl::scanCompleteCallback, this))
    , sub_apriltag_(nh->subscribe("/tag_detections", 1000, &DisinfectionControl::tagDetectionCallback, this))
    , sub_map_pose_(nh->subscribe("/map_pose", 1000, &DisinfectionControl::mapPoseCallback, this))
{
}


void DisinfectionControl::publishRobotState(int state)
{
    std_msgs::Int8 msg;
    msg.data = state;
    pub_robot_state_.publish(msg);
}


void DisinfectionControl::tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
    // Iterate through all detected tags
    for (const auto& detection : msg.detections)
    {
        int tag_id = detection.id[0];

        // If a workspace is detected
        if (tag_id >= 50)
        {
            // Set the robot state to scanning
            robot_state_ = 2;
            publishRobotState(robot_state_);
            addDetection(tag_id, detection);
        }

        if (tag_id < 100 && robot_state_ == 2)
        {
            addDetection(tag_id, detection);
            // if (detected_people.find(detection.id) == detected_people.end())
            // {
            //     detected_people.insert(detection.tag_id, person);
            // }
            // else
            // {
            //
            // }
        }
    }
}


void DisinfectionControl::addDetection(int tag_id, const apriltag_ros::AprilTagDetection& apriltag_msg)
{
    std::vector<float> location;
    location.reserve(3);

    // geometry_msgs::PoseStamped msg;
    // msg.header.frame_id =
    // msg.pose.position.x = current_map_pose.pose.position.x + apriltag_msg.pose.pose.pose.position.x;
    // msg.pose.position.y = current_map_pose.pose.position.y + apriltag_msg.pose.pose.pose.position.y;
    // msg.pose.position.z = current_map_pose.pose.position.z + apriltag_msg.pose.pose.pose.position.z;
    // msg.pose.orientation.x = current_map_pose.pose.orientation.x + apriltag_msg.pose.pose.pose.orientation.x;
    // msg.pose.orientation.y = current_map_pose.pose.orientation.y + apriltag_msg.pose.pose.pose.orientation.y;
    // msg.pose.orientation.z = current_map_pose.pose.orientation.z + apriltag_msg.pose.pose.pose.orientation.z;
    // msg.pose.orientation.w = current_map_pose.pose.orientation.w + apriltag_msg.pose.pose.pose.orientation.w;

    location[0] = current_map_pose.pose.position.x + apriltag_msg.pose.pose.pose.position.x;
    location[1] = current_map_pose.pose.position.y + apriltag_msg.pose.pose.pose.position.y;
    location[2] = current_map_pose.pose.position.z + apriltag_msg.pose.pose.pose.position.z;

    std::cout << "tag_id: " << tag_id << "\t";
    std::cout << "location: " << location[0] << "_ " << location[1] << "_ " << location[2];
    std::cout << std::endl;

    std::pair<int,std::vector<float>> pair = std::make_pair(tag_id, location);
    // detections_.insert(std::make_pair<int,std::vector<float>>(tag_id, location));
    detections_.insert(pair);
}


void DisinfectionControl::scanCompleteCallback(const std_msgs::Bool& msg)
{
    robot_state_ = 0;

    for (auto& it : detections_)
    {
        int tag_id = it.first;
        std::vector<float> location = it.second;
        std::cout << "location_size: " << location.size() << std::endl;

        // Seg faulting?
        // std::cout << "tag_id: " << tag_id << "\t";
        // std::cout << "location: " << location[0] << "_ " << location[1] << "_ " << location[2];
        // std::cout << std::endl;
    }

    // scan_complete_ = true;
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


void DisinfectionControl::mapPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    current_map_pose = msg;
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

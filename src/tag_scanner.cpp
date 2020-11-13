/**
 * tag_scanner.cpp
 *
 * Node for scanning April tags corresponding to people and workspaces. When a workspace is detected, a scan is
 * performed for people working at the workspace.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#include "turtlebot3_disinfection/tag_scanner.h"


/**
 * Constructs a class instance and initialises publishers and subscribers.
 *
 * @param nh Pointer to a ROS NodeHandle instance. Used to initialise publishers and subscribers.
 */
TagScanner::TagScanner(ros::NodeHandle* nh)
    : pub_robot_state_(nh->advertise<std_msgs::Int8>("/robot_state", 10))
    , pub_tag_scan_(nh->advertise<turtlebot3_disinfection::Scan>("/tag_scan", 10))
    , sub_rotation_complete_(nh->subscribe("/rotation_complete", 1000, &TagScanner::scanCompleteCallback, this))
    , sub_apriltag_(nh->subscribe("/tag_detections", 1000, &TagScanner::tagDetectionCallback, this))
    , sub_map_pose_(nh->subscribe("/map_pose", 1000, &TagScanner::mapPoseCallback, this))
{
}

/**
 * Publishes a robot state to the /robot_state topic to request a change in behaviour in the motion_planner node.
 *
 * @param state Integer value representing the robot state. The values are
 */
void TagScanner::publishRobotState(int state)
{
    std_msgs::Int8 msg;
    msg.data = state;
    pub_robot_state_.publish(msg);
}

/**
 * Publishes a robot state to the /robot_state topic to request a change in behaviour in the motion_planner node.
 *
 * @param state Integer value representing the robot state.
 */
void TagScanner::tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
    // Iterate through all detected tags
    for (const auto& detection : msg.detections)
    {
        int tag_id = detection.id[0];

        // If a workspace is detected
        if (tag_id >= 50)
        {
            // Set the robot state to scanning
            robot_state_ = MotionPlanner::RobotState::TAG_SCANNING;
            publishRobotState(robot_state_);
            addDetection(tag_id, detection);
        }

        if (tag_id < 50 && robot_state_ == MotionPlanner::RobotState::STOPPED)
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


void TagScanner::addDetection(int tag_id, const apriltag_ros::AprilTagDetection& apriltag_msg)
{
    geometry_msgs::PoseStamped msg;
    msg.header = apriltag_msg.pose.header;
    msg.pose.position.x = current_map_pose.pose.position.x + apriltag_msg.pose.pose.pose.position.x;
    msg.pose.position.y = current_map_pose.pose.position.y + apriltag_msg.pose.pose.pose.position.y;
    msg.pose.position.z = current_map_pose.pose.position.z + apriltag_msg.pose.pose.pose.position.z;
    msg.pose.orientation.x = current_map_pose.pose.orientation.x + apriltag_msg.pose.pose.pose.orientation.x;
    msg.pose.orientation.y = current_map_pose.pose.orientation.y + apriltag_msg.pose.pose.pose.orientation.y;
    msg.pose.orientation.z = current_map_pose.pose.orientation.z + apriltag_msg.pose.pose.pose.orientation.z;
    msg.pose.orientation.w = current_map_pose.pose.orientation.w + apriltag_msg.pose.pose.pose.orientation.w;

    ROS_INFO("tag_id: %d\tposition: %.3f %.3f %.3f", tag_id, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    std::pair<int,geometry_msgs::PoseStamped> pair = std::make_pair(tag_id, msg);
    detections_.insert(pair);
}


void TagScanner::scanCompleteCallback(const std_msgs::Empty& msg)
{
    // Robot is in wall following state


    turtlebot3_disinfection::Scan scan_msg;
    scan_msg.scan_end_time.data = ros::Time::now();

    for (auto& it : detections_)
    {
        int tag_id = it.first;
        geometry_msgs::PoseStamped& pose_msg = it.second;

        if (tag_id >= 50)
        {
            scan_msg.workspace_id = tag_id;
            scan_msg.workspace_pose = pose_msg;
        }
        else
        {
            scan_msg.people_ids.push_back(tag_id);
            scan_msg.people_poses.push_back(pose_msg);
        }
    }

    // If no people were detected in the workspace
    if (scan_msg.people_ids.size() == 0)
    {
        // Sleep to represent disinfecting
        ros::Duration(4).sleep();
        scan_msg.disinfected = true;
    }
    else
    {
        scan_msg.disinfected = false;
    }

    // Return to wall following
    robot_state_ = MotionPlanner::RobotState::WALL_FOLLOWING;
    publishRobotState(robot_state_);

    pub_tag_scan_.publish(scan_msg);

    // Clear the stored detections from the current scan
    detections_.clear();
}


void TagScanner::mapPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    current_map_pose = msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_scanner");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    TagScanner tag_scanner = TagScanner(&nh);

    ros::spin();

    return 0;
}

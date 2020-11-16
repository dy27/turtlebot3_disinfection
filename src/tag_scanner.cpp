/**
 * tag_scanner.cpp
 *
 * Node for scanning April tags corresponding to people and workspaces. When a workspace is detected, a scan is
 * performed to detect people working at the workspace.
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
void TagScanner::publishRobotState(int state) const
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
            // Calculate the distance of the April tag from the camera
            float sum_squares = std::pow(detection.pose.pose.pose.position.x, 2)
                + std::pow(detection.pose.pose.pose.position.y, 2)
                + std::pow(detection.pose.pose.pose.position.z, 2);
            float distance = std::sqrt(sum_squares);
            // ROS_INFO("xyz: %f %f %f \tTAG DISTANCE: %f", detection.pose.pose.pose.position.x,
            //     detection.pose.pose.pose.position.y, detection.pose.pose.pose.position.z, distance);

            // Skip this workspace if it is too far away
            if (distance > 0.45)
            {
                continue;
            }

            // If this detection was the last workspace to be scanned
            if (last_workspace_scanned_id == tag_id)
            {
                // If the workspace was scanned already scanned in the last 20 seconds
                ros::Duration diff = detection.pose.header.stamp - last_workspace_scanned_time;
                if (diff < ros::Duration(30))
                {
                    // Skip scanning this workspace
                    continue;
                }
            }
            // Set the robot state to scanning
            robot_state_ = MotionPlanner::RobotState::TAG_SCANNING;
            publishRobotState(robot_state_);

            // Add the detection
            addDetection(tag_id, detection);

            // Update the last scanned workspace to this detection
            last_workspace_scanned_id = tag_id;
            last_workspace_scanned_time = detection.pose.header.stamp;
        }

        // If a person is detected and the robot is in scanning mode, add the detection
        if (tag_id < 50 && robot_state_ == MotionPlanner::RobotState::TAG_SCANNING)
        {
            addDetection(tag_id, detection);
        }
    }
}

/**
 * Add a tag detection to the array of detections stored for the current scan.
 *
 * @param tag_id The ID of the tag to be added.
 * @param apriltag_msg The AprilTagDetection msg corresponding to the detected tag.
 */
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

    detections_.insert(std::make_pair(tag_id, msg));
}

/**
 * Callback function from the motion_planner node which is called when a full rotation has been performed for the scan.
 * This function converts all the detections found during the scan into a Scan msg to send to the database node.
 *
 * @param msg Empty msg. Receving this triggers the callback, there is no need for any information to be stored.
 */
void TagScanner::scanCompleteCallback(const std_msgs::Empty& msg)
{
    turtlebot3_disinfection::Scan scan_msg;
    scan_msg.scan_end_time.data = ros::Time::now();

    // Iterate through all detections in the scan
    for (auto& it : detections_)
    {
        // Get the tag_id and pose of the detection
        int tag_id = it.first;
        geometry_msgs::PoseStamped& pose_msg = it.second;

        // If the detected tag is a workspace, set the workspace to be this workspace in the Scan msg
        if (tag_id >= 50)
        {
            scan_msg.workspace_id = tag_id;
            scan_msg.workspace_pose = pose_msg;
        }
        // If the detected tag is a person, add the tag and pose to the Scan msg
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
        ros::Duration(3).sleep();
        scan_msg.disinfected = true;
    }
    else
    {
        scan_msg.disinfected = false;
    }

    // Return to wall following
    robot_state_ = MotionPlanner::RobotState::WALL_FOLLOWING;
    publishRobotState(robot_state_);

    // Publish the Scan msg
    pub_tag_scan_.publish(scan_msg);

    // Clear the stored detections from the current scan
    detections_.clear();
}

/**
 * Callback function which is called when the robot pose is updated on the /map_pose topic. This function updates the
 * current_map_pose value stored in this class.
 *
 * @param msg PoseStamped msg containing the pose of the robot relative to the map frame.
 */
void TagScanner::mapPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    current_map_pose = msg;
}


int main(int argc, char **argv)
{
    // Initialise the node
    ros::init(argc, argv, "tag_scanner");

    // Create a node handle to pass to the class constructor
    ros::NodeHandle nh;

    // Instantiate the class
    TagScanner tag_scanner = TagScanner(&nh);

    // Wait for and process callbacks
    ros::spin();

    return 0;
}

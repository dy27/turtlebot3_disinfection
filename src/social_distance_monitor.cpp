/**
 * social_distance_monitor.cpp
 *
 * This node subscribes to all April tag detections and monitors distances between people to enforce social distancing.
 * An alert is issued when a pair of people violate the distancing requirements.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#include "turtlebot3_disinfection/social_distance_monitor.h"

/**
 * Class constructor which initialises subscribers and sets the separation distance.
 *
 * @param nh ROS NodeHandle object used to instantiate this class as a ROS node.
 */
SocialDistanceMonitor::SocialDistanceMonitor(ros::NodeHandle* nh, float separation_distance)
    : sep_distance_(separation_distance)
    , sub_apriltag_(nh->subscribe("/tag_detections", 1000, &SocialDistanceMonitor::tagDetectionCallback, this))
{
}

/**
 * Callback function which is called when an array of April tag detections is received. This function then iterates
 * through the tags that correspond to people and finds the distance between each pair of people.
 *
 * @param msg AprilTagDetectionArray msg containing an array of detected April tags from a single camera frame.
 */
void SocialDistanceMonitor::tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg) const
{
    // Use a vector of pairs to store each person's ID and pose
    std::vector<std::pair<int,geometry_msgs::Pose>> people_poses;
    people_poses.reserve(msg.detections.size());

    // Iterate through all detections
    for (auto& detection : msg.detections)
    {
        // Get the tag_id from the detection message
        int tag_id = detection.id[0];

        // Get the pose from the detection message
        geometry_msgs::Pose pose = detection.pose.pose.pose;

        // If the tag corresponds to a person
        if (tag_id < 50)
        {
            // Add the person to the vector
            people_poses.push_back(std::make_pair(tag_id, pose));
        }
    }

    // Get the number of people that were found
    int n_people = people_poses.size();

    // Iterate through all combinations of two people in the vector
    float distance;
    for (int i=0; i<n_people; i++)
    {
        for (int j=i+1; j<n_people; j++)
        {
            // Calculate the distance between the two people
            distance = calculateDistance(people_poses[i].second, people_poses[j].second);

            // Print the distance
            std::cout << "Person " << people_poses[i].first << ", Person " << people_poses[j].first
                << "\tDistance: " << distance << " metres";

            // If the distance is less than the separation_distance
            if (distance < sep_distance_)
            {
                // Issue an alert
                std::cout << "\t<---ALERT";
            }
            std::cout << std::endl;
        }
    }
}

/**
 * Calculates the Euclidean distance between two poses in 3D space.
 *
 * @param msg1 The first Pose msg to use in the distance calculation.
 * @param msg2 The second Pose msg to use in the distance calculation.
 * @return The distance between the two poses.
 */
float SocialDistanceMonitor::calculateDistance(const geometry_msgs::Pose& msg1,
    const geometry_msgs::Pose& msg2) const
{
    // Calculate the sum of square differences
    float sum_square_diffs = std::pow(msg1.position.x - msg2.position.x, 2)
                           + std::pow(msg1.position.y - msg2.position.y, 2)
                           + std::pow(msg1.position.z - msg2.position.z, 2);

    // Calculat the square root of sum_square_diffs to get the distance
    float distance = std::sqrt(sum_square_diffs);

    return distance;
}


int main(int argc, char **argv)
{
    // Separation distance threshold
    const float separation_distance = 0.1f;

    // Initialise the node
    ros::init(argc, argv, "social_distance_monitor");

    // Create a node handle to pass to the class constructor
    ros::NodeHandle nh;

    // Instantiate the class
    SocialDistanceMonitor social_distance_monitor = SocialDistanceMonitor(&nh, separation_distance);

    // Wait for and process callbacks
    ros::spin();

    return 0;
}

// publish to /social_distance_alerts

#include "turtlebot3_disinfection/social_distance_monitor.h"


SocialDistanceMonitor::SocialDistanceMonitor(ros::NodeHandle* nh, float separation_distance)
    : sep_distance_(separation_distance)
    // , sub_tag_scan_(nh->subscribe("/tag_scan", 1000, &SocialDistanceMonitor::tagScanCallback, this))
    , sub_apriltag_(nh->subscribe("/tag_detections", 1000, &SocialDistanceMonitor::tagDetectionCallback, this))
{
}


// void SocialDistanceMonitor::tagScanCallback(const turtlebot3_disinfection::Scan& msg)
// {
//     int n_people = msg.people_ids.size();
//
//     float distance;
//     for (int i=0; i<n_people; i++)
//     {
//         for (int j=i+1; j<n_people; j++)
//         {
//             distance = calculateDistance(msg.people_poses[i], msg.people_poses[j]);
//             if (distance < sep_distance_)
//             {
//                 // Issue alert
//             }
//             ROS_INFO("Person %d and Person %d are %.3f metres apart", i, j, distance);
//         }
//     }
// }

void SocialDistanceMonitor::tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg)
{
    std::vector<std::pair<int,geometry_msgs::Pose>> people_poses;
    people_poses.reserve(msg.detections.size());

    for (auto& detection : msg.detections)
    {
        int tag_id = detection.id[0];
        geometry_msgs::Pose pose = detection.pose.pose.pose;

        if (tag_id < 50)
        {
            // std::pair =
            people_poses.push_back(std::make_pair(tag_id, pose));
        }
    }

    int n_people = people_poses.size();

    float distance;
    for (int i=0; i<n_people; i++)
    {
        for (int j=i+1; j<n_people; j++)
        {
            distance = calculateDistance(people_poses[i].second, people_poses[j].second);
            if (distance < sep_distance_)
            {
                // Issue alert
            }
            ROS_INFO("Person %d and Person %d are %.3f metres apart", people_poses[i].first, people_poses[j].first, distance);
        }
    }
}


float SocialDistanceMonitor::calculateDistance(const geometry_msgs::Pose& msg1,
    const geometry_msgs::Pose& msg2)
{
    float sum_square_diffs = std::pow(msg1.position.x - msg2.position.x, 2)
        + std::pow(msg1.position.y - msg2.position.y, 2)
        + std::pow(msg1.position.z - msg2.position.z, 2);

    float distance = std::sqrt(sum_square_diffs);

    return distance;
}


// void SocialDistanceMonitor::sendAlert()

int main(int argc, char **argv)
{
    const float separation_distance = 0.1f;

    ros::init(argc, argv, "social_distance_monitor");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    SocialDistanceMonitor social_distance_monitor = SocialDistanceMonitor(&nh, separation_distance);

    ros::spin();

    return 0;
}

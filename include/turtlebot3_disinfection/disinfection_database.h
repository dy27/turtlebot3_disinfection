#ifndef DISINFECTION_DATABASE_
#define DISINFECTION_DATABASE_

#include "ros/ros.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>
#include <turtlebot3_disinfection/Scan.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/thread/thread.hpp>

struct Location
{
    float x;
    float y;
    float z;
};


struct DetectionObject
{
    int tag_id;
    std::vector<ros::Time> detection_times;
    std::vector<Location> detection_locations;

    DetectionObject(int id) : tag_id(id) {}
};


struct Person : DetectionObject
{
    std::string name;
    int last_seen_workspace_id;

    Person(int person_id)
        : DetectionObject(person_id)
        , name("Person" + std::to_string(person_id))
        , last_seen_workspace_id(-1)
    {
    }
};


struct Workspace : DetectionObject
{
    std::vector<ros::Time> disinfection_times;

    Workspace(int workspace_id) : DetectionObject(workspace_id) {}
};


// struct ScanEntry
// {
//     const int workspace_id;
//     const Location workspace_location;
//
//     const std::vector<int> people_ids;
//     const std::vector<Location> people_locations;
//
//     const ros::Time scan_end_time;
//
//     const bool disinfected;
//
//     ScanEntry(const turtlebot3_disinfection::Scan& msg)
//         : workspace_id(msg.workspace_id)
//         , workspace_location({(float)msg.workspace_pose.pose.position.x,
//                               (float)msg.workspace_pose.pose.position.y,
//                               (float)msg.workspace_pose.pose.position.z})
//         , people_ids(msg.people_ids)
//         , people_locations(getPeopleLocations(msg))
//         , scan_end_time(msg.scan_end_time.data)
//         , disinfected(msg.disinfected)
//     {
//     }
//
//     std::vector<Location> getPeopleLocations(const turtlebot3_disinfection::Scan& msg)
//     {
//         std::vector<Location> locations;
//         locations.reserve(msg.people_ids.size());
//
//         for (auto& pose_msg : msg.people_poses)
//         {
//             locations.push_back({pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z});
//         }
//
//         return locations;
//     }
// };


class DisinfectionDatabase
{
public:
    DisinfectionDatabase(ros::NodeHandle* nh);

    ~DisinfectionDatabase();

    void printWorkspaces();

    void printPeople();

    void printScans();

    void tagScanCallback(const turtlebot3_disinfection::Scan& msg);

private:

    std::unordered_map<int,Workspace*> workspace_database_;
    std::unordered_map<int,Person*> person_database_;

    // std::vector<ScanEntry> scan_list_;

    std::vector<turtlebot3_disinfection::Scan> scan_list_;

    const ros::Subscriber sub_tag_scan_;
};

#endif

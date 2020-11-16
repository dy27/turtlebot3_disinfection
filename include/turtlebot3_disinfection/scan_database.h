/**
 * scan_database.h
 *
 * Database node to store detections of people and workspaces. Information is sent to the database node through the
 * /tag_scan topic using a Scan msg format.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#ifndef SCAN_DATABASE_
#define SCAN_DATABASE_

#include "ros/ros.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>
#include <turtlebot3_disinfection/Scan.h>
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


class ScanDatabase
{
    public:
        ScanDatabase(ros::NodeHandle* nh);

        ~ScanDatabase();

        void printWorkspaces() const;

        void printPeople() const;

        void printScans();

        void tagScanCallback(const turtlebot3_disinfection::Scan& msg);

    private:

        std::unordered_map<int,Workspace*> workspace_database_;

        std::unordered_map<int,Person*> person_database_;

        std::vector<turtlebot3_disinfection::Scan> scan_list_;

        const ros::Subscriber sub_tag_scan_;
};

#endif

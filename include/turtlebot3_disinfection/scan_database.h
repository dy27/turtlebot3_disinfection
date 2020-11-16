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

// Location object to represent coordinate locations on the map.
struct Location
{
    float x;
    float y;
    float z;
};

// Base class for objects which are detectable using April tags.
struct DetectionObject
{
    int tag_id;
    std::vector<ros::Time> detection_times;
    std::vector<Location> detection_locations;

    DetectionObject(int id) : tag_id(id) {}
};

// Stores information about a person.
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

// Stores information about a workspace.
struct Workspace : DetectionObject
{
    std::vector<ros::Time> disinfection_times;

    Workspace(int workspace_id) : DetectionObject(workspace_id) {}
};


class ScanDatabase
{
    public:
        // Class constructor.
        ScanDatabase(ros::NodeHandle* nh);

        // Class destructor which frees dynamic memory allocations used in the database.
        ~ScanDatabase();

        // Prints the list of all workspaces in the database and details about each workspace.
        void printWorkspaces() const;

        // Prints the list of all people in the database and details about each person.
        void printPeople() const;

        // Prints the list of all scans in the database and details about each scan.
        void printScans();

        // Callback function which adds received objects to the database.
        void tagScanCallback(const turtlebot3_disinfection::Scan& msg);

    private:
        // Map container of workspaces.
        std::unordered_map<int,Workspace*> workspace_database_;

        // Map container of people.
        std::unordered_map<int,Person*> person_database_;

        // List of scans.
        std::vector<turtlebot3_disinfection::Scan> scan_list_;

        // Subscriber to the Scan messages sent from the tag_scanner node.
        const ros::Subscriber sub_tag_scan_;
};

#endif

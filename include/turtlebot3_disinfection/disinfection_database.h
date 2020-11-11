#ifndef DISINFECTION_CONTROL_
#define DISINFECTION_CONTROL_

#include "ros/ros.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>
#include <apriltags_ros/AprilTagDetectionArray.h>


class DetectionObject
{
public:
    int tag_id;
    ros::Time last_detection_time;
};


class Person : DetectionObject
{
public:
    std::string name;
};


class Workspace : DetectionObject
{
public:
    ros::Time last_disinfection_time;
};


class Scan
{
public:
    Workspace* workspace;
    std::unordered_map<int,Person*> people;
    ros::Time scan_end_time;
    // std::vector<Person> people;
};


class DisinfectionDatabase
{
public:
    DisinfectionControl(ros::NodeHandle* nh);

    void printLog();

    void publishDatabase(const ros::Publisher& )

    void addScanEntry(int workspace_id, std::vector<int> people_ids, ros::Time scan_end_time);

private:

    std::unordered_map<int,Workspace> workspace_database;
    std::unordered_map<int,Person> people_database;

    std::vector<Scan> scan_list;

    const std::unordered_map<int,std::string> tag_id_names;

    const ros::Subscriber sub_apriltags_;
};

#endif

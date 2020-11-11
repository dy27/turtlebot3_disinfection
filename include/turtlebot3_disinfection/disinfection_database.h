#ifndef DISINFECTION_CONTROL_
#define DISINFECTION_CONTROL_

#include "ros/ros.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>
#include <apriltags_ros/AprilTagDetectionArray.h>


struct DetectionObject
{
    int tag_id;
    ros::Time last_detection_time;
};


struct Person : DetectionObject
{
    std::string name;
};


struct Workspace : DetectionObject
{
    ros::Time last_disinfection_time;
};

class Scan
{
public:
    Workspace workspace;
    std::unordered_map<int, Person> people;
    // std::vector<Person> people;
};

class DisinfectionDatabase
{
    public:
        DisinfectionControl(ros::NodeHandle* nh);

        void tagDetectionCallback(const apriltags_ros::AprilTagDetectionArray& msg);

        void printLog();

        void publishDatabase(const ros::Publisher& )

    private:

        std::unordered_map<int, DetectionObject> objects;

        const std::unordered_map<int, std::string> tag_id_names;

        // const ros::Publisher pub_vel_;

        const ros::Subscriber sub_apriltags_;
};

#endif

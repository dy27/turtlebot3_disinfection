#ifndef DISINFECTION_CONTROL_
#define DISINFECTION_CONTROL_

#include "ros/ros.h"
// #include "disinfection_database.h"

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <std_msgs/Bool.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>
#include <std_msgs/Int8.h>


class DisinfectionControl
{
    public:
        DisinfectionControl(ros::NodeHandle* nh);

        void publishRobotState(int state);

        void publishScanComplete(bool scan_complete);

        void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray& msg);

        void loop();

    private:
        int robot_state_;

        int scan_count_;

        // Workspace* detected_workspace;
        // // std::vector<Person> detected_people;
        // std::unordered_map<int, Person>* detected_people;

        const ros::Publisher pub_robot_state_;

        const ros::Publisher pub_scan_complete_;

        const ros::Subscriber sub_apriltag_;
};

#endif

/**
 * ... text ...
 */

#include "turtlebot3_disinfection/motion_planner.h"

#define WALL_DIST 0.15f
#define WALL_TRACK_DIST 0.4f
#define MAX_LIN_VEL 0.22f
#define MAX_ANG_VEL 2.84f
#define P_LINEAR 0.1f
#define P_ANGULAR 0.0f
#define D_LINEAR 0.1f
#define D_ANGULAR 0.5f

#define FRONT 0
#define LEFT 90
#define LEFT_FRONT 80
#define LEFT_BACK 100

#define MAX_RANGE 3.5f


// const int SCAN_RANGES = {0, }

MotionPlanner::MotionPlanner(ros::NodeHandle* nh)
    : mode_(0)
    , pub_vel_(nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10))
    , sub_laser_(nh->subscribe("/scan", 1000, &MotionPlanner::laserCallback, this))
{
    // pub_vel_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // sub_laser_ = nh->subscribe("/scan", 1000, &MotionPlanner::laserCallback, this);
}

void MotionPlanner::publishVelocity(const float lin_vel, const float ang_vel)
{
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = lin_vel;
    vel_msg.linear.y = 0.0f;
    vel_msg.linear.z = 0.0f;
    vel_msg.angular.x = 0.0f;
    vel_msg.angular.y = 0.0f;
    vel_msg.angular.z = ang_vel;

    pub_vel_.publish(vel_msg);
}

float MotionPlanner::getRange(const sensor_msgs::LaserScan& msg, int index)
{
    std::vector<float> ranges;
    ranges.reserve(5);
    float range;
    for (int i=0; i<5; i++)
    {
        range = msg.ranges[index-2];
        if (range < 0.001 || range > msg.range_max)
        {
            range = MAX_RANGE;
        }
        ranges[i] = range;
    }

    return median(ranges);

    // float range = msg.ranges[index];
    // if (range < 0.001 || range > msg.range_max)
    // {
    //     range = MAX_RANGE;
    // }


    // do median

    return range;
}

float MotionPlanner::median(std::vector<float>& distances)
{
    std::sort(distances.begin(), distances.end());
    int index = distances.size() / 2;
    if (distances.size() % 2 == 0)
    {
        return (distances[index] + distances[index+1]) / 2;
    }
    else
    {
        return distances[index];
    }
}

float MotionPlanner::mapToRange(float value, const std::vector<float>& range, const std::vector<float>& new_range,
    bool saturate=true)
{
    // if (value < range[0] && value > range[1])
    // {
    //     ROS_INFO("[WARN] mapToRange: Value out of range");
    // }
    float mapped_value;

    if (value < range[0] && saturate)
    {
        mapped_value = new_range[0];
    }
    else if (value > range[1] && saturate)
    {
        mapped_value = new_range[1];
    }
    else
    {
        float percent = (value - range[0]) / (range[1] - range[0]);

        mapped_value = new_range[0] + percent * (new_range[1] - new_range[0]);
    }

    // ROS_ASSERT_MSG(value < range[0] && value > range[1],
    //     "Value %.3f outside of range [%.3f,%.3f]", value, range[0], range[1]);

    return mapped_value;
}

float MotionPlanner::getMinRange(const sensor_msgs::LaserScan& msg, int start_angle, int end_angle)
{
    while (start_angle > end_angle)
    {
        end_angle += 360;
    }

    float min_range = MAX_RANGE + 1;
    for (int i=start_angle; i<=end_angle; i++)
    {
        float range = getRange(msg, i % 360);
        // ROS_INFO("r: %.3f\n", range);
        if (range < min_range)
        {
            min_range = range;
        }
    }

    return min_range;
}

void MotionPlanner::laserCallback(const sensor_msgs::LaserScan& msg)
{
    float lin_vel, ang_vel;

    // positive -> turn left
    // float p_error = (getRange(msg, LEFT) - WALL_DIST) / getRange(msg, LEFT);
    //
    // // positive -> turn left
    // float d_error = (getRange(msg, LEFT_FRONT) - getRange(msg, LEFT_BACK)) /
    //                 (getRange(msg, LEFT_FRONT) + getRange(msg, LEFT_BACK));
    //
    // float lin_vel = std::max(std::min(D_LINEAR / d_error, MAX_LIN_VEL), -MAX_LIN_VEL);
    // float lin_vel = std::max(std::min(D_LINEAR / d_error, MAX_LIN_VEL), -MAX_LIN_VEL);
    //
    // float ang_vel = std::max(std::min(P_ANGULAR * p_error + D_ANGULAR * d_error, MAX_ANG_VEL), -MAX_ANG_VEL);

    int i_min = 60;
    int i_max = 120;

    int min_index;
    float min_range = MAX_RANGE + 1;
    for (int i=i_min; i<i_max; i++)
    {
        float range = getRange(msg, i);
        // ROS_INFO("r: %.3f\n", range);
        if (range < min_range)
        {
            min_index = i;
            min_range = range;
        }
    }


    float dist_error = min_range - WALL_DIST;

    ROS_INFO("range: %.3f\tdiff: %.3f", min_range, dist_error);

    if (dist_error > 0.03)
    {
        ang_vel = MAX_ANG_VEL/2;
    }
    else if (dist_error < -0.03)
    {
        ang_vel = -MAX_ANG_VEL/2;
    }
    else
    {
        ang_vel = 0;
    }


    float angle_error = getRange(msg, LEFT_FRONT) - getRange(msg, LEFT_BACK);

    if (getRange(msg, LEFT_BACK) > 0.3f)
    {
        angle_error = 0.0f;
    }

    ROS_INFO("leftfront: %.3f\tleftback: %.3f\terr: %.3f", getRange(msg, LEFT_FRONT), getRange(msg, LEFT_BACK), angle_error);
    if (angle_error > 0.05)
    {
        lin_vel = MAX_LIN_VEL/2;
        ang_vel = MAX_ANG_VEL;
    }
    else if (angle_error < -0.05)
    {
        lin_vel = MAX_LIN_VEL;
        ang_vel = -MAX_ANG_VEL/5;
    }
    else
    {
        lin_vel = MAX_LIN_VEL;
        ang_vel = 0;
    }

    // lin_vel = 0.22;



    // const std::vector<float> error_range = {0.0f, 0.2f};
    //
    // const std::vector<float> ang_vel_range = {0.0f, MAX_ANG_VEL};

    // const std::vector<float> lin_vel_range = {}

    // const float ANGLE_ERROR_WEIGHT = 0.5f;
    // const float DIST_ERROR_WEIGHT = 0.5f;

    const float ANGLE_ERROR_WEIGHT = 1.0f;
    const float DIST_ERROR_WEIGHT = 1.0f;

    const std::vector<float> ang_error_range = {-0.15f, 0.15f};

    const std::vector<float> ang_vel_range = {-ANGLE_ERROR_WEIGHT * MAX_ANG_VEL, ANGLE_ERROR_WEIGHT * MAX_ANG_VEL};

    const std::vector<float> dist_error_range = {-0.1f, 0.1f};

    const std::vector<float> dist_vel_range = {-DIST_ERROR_WEIGHT * MAX_ANG_VEL, DIST_ERROR_WEIGHT * MAX_ANG_VEL};

    float dist_ang_vel = mapToRange(dist_error, dist_error_range, dist_vel_range);
    ROS_INFO("dist error %.3f mapped to dist_ang_vel %.3f", dist_error, dist_ang_vel);

    ang_vel = mapToRange(angle_error, ang_error_range, ang_vel_range);
    ROS_INFO("ang error %.3f mapped to ang_ang_vel %.3f", angle_error, ang_vel);

    ang_vel += dist_ang_vel;

    if (ang_vel > MAX_ANG_VEL)
    {
        ang_vel = MAX_ANG_VEL;
    }
    else if (ang_vel < -MAX_ANG_VEL)
    {
        ang_vel = -MAX_ANG_VEL;
    }

    lin_vel = MAX_LIN_VEL;

    if (min_range > WALL_TRACK_DIST)
    {
        ang_vel = 0;
    }


    // lin_vel = 0;

    float min_range_front = getMinRange(msg, 345, 15);
    ROS_INFO("min_range_front: %.3f", min_range_front);
    if (min_range_front < WALL_DIST + 5)
    {
        lin_vel = 0;
        ang_vel = -MAX_ANG_VEL;
    }

    publishVelocity(lin_vel, ang_vel);

    // float front_range = msg.ranges[0];
    // float left_range = msg.ranges[90];
    //
    // if (front_range > WALL_DIST && left_range > WALL_DIST)
    // {
    //     publishVelocity(0.6 * MAX_LIN_VEL, -0.3 * MAX_ANG_VEL);
    // }
    // else if (front_range < WALL_DIST && left_range > WALL_DIST)
    // {
    //     publishVelocity(0.0f, 0.2 * MAX_ANG_VEL);
    // }
    // else if (front_range > WALL_DIST && left_range < WALL_DIST)
    // {
    //     publishVelocity(0.6 * MAX_LIN_VEL, 0.0f);
    // }
    // else
    // {
    //     publishVelocity(0.0f, 0.6 * MAX_ANG_VEL);
    // }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_planner");

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    MotionPlanner motion_planner = MotionPlanner(&nh);

    // ros::Rate loop_rate(10);

    // initialise 0-vectors
    // std::vector<float> position(3, 0);
    // std::vector<float> orientation(4, 0);
    // position[0] = 1;
    // orientation[3] = 1;
    //
    // motion_planner.publishWaypoint(position, orientation);

    ros::spin();

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //
    //
    //     // motion_planner.updateWaypoint();
    //
    //
    //     loop_rate.sleep();
    // }

    return 0;
}

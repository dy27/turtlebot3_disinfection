/**
 * motion_planner.cpp
 *
 * This node controls all navigation and movement of the robot by publishing to the /cmd_vel topic.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#include "turtlebot3_disinfection/motion_planner.h"

/**
 * Class constructor which initialises publishers and subscribers, and loads parameters.
 *
 * @param nh ROS NodeHandle object used to instantiate this class as a ROS node.
 */
MotionPlanner::MotionPlanner(ros::NodeHandle* nh)
    : robot_state_(RobotState::WALL_FOLLOWING)
    , pub_vel_(nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10))
    , sub_laser_(nh->subscribe("/scan", 1000, &MotionPlanner::laserCallback, this))
    , sub_robot_state_(nh->subscribe("/robot_state", 1000, &MotionPlanner::robotStateCallback, this))
    , pub_rotation_complete_(nh->advertise<std_msgs::Empty>("/rotation_complete", 10))
    , WALL_DIST(getParam<float>(nh, "wall_distance"))
    , FRONT_TURN_DIST(getParam<float>(nh, "front_turn_distance"))
    , WALL_MAX_TRACK_DIST(getParam<float>(nh, "wall_track_distance"))
    , MAX_LIN_VEL(getParam<float>(nh, "max_linear_velocity"))
    , MAX_ANG_VEL(getParam<float>(nh, "max_angular_velocity"))
    , MAX_RANGE(getParam<float>(nh, "max_scan_range"))
    , FRONT_RANGE(getParam<std::vector<int>>(nh, "front_angle_range"))
    , LEFT_RANGE(getParam<std::vector<int>>(nh, "left_angle_range"))
    , LEFT_FRONT_RANGE(getParam<std::vector<int>>(nh, "left_front_angle_range"))
    , LEFT_BACK_RANGE(getParam<std::vector<int>>(nh, "left_back_angle_range"))
    , ANGLE_ERROR_RANGE(getParam<std::vector<float>>(nh, "angle_error_range"))
    , DIST_ERROR_RANGE(getParam<std::vector<float>>(nh, "distance_error_range"))
    , ANG_VEL_RANGE({-MAX_ANG_VEL, MAX_ANG_VEL})
{
}

/**
 * Publishes to the /cmd_vel topic to make the robot move at the specified linear and angular velocities.
 *
 * @param lin_vel Linear velocity in metres per second.
 * @param ang_vel Angular velocity in radians per second.
 */
void MotionPlanner::publishVelocity(float lin_vel, float ang_vel) const
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

/**
 * Publishes to the /rotation_complete topic to signal that a full rotation has been performed by the robot.
 */
void MotionPlanner::publishScanComplete() const
{
    std_msgs::Empty msg;
    pub_rotation_complete_.publish(msg);
}

/**
 * Returns the distance at a certain index in the array of measurements from the laser. For the turtlebot, the index is
 * equivalent to the angle in degrees as each index increment corresponds to 1 degree.
 * The distance at a certain angle/index is returned as the median distance of its distance value and the distances of
 * a number of adjacent measurements, determined by the n_measurements parameter.
 *
 * e.g. If the function is called with an index/angle of 10 with the default value n_measurements=5, the median of the
 * 5 measurements in the range of [8,12] is returned.
 *
 * @param msg LaserScan msg received in the laserCallback callback function.
 * @param int index Index of the distance measurement in the array of measurements from the laser sensor.
 * @param n_measurements Number of adjacent measurements to use for the calculation of the median distance.
 * @return The distance measurement at the specified angle.
 */
float MotionPlanner::getRange(const sensor_msgs::LaserScan& msg, int index, int n_measurements=5) const
{
    std::vector<float> measurements;
    measurements.reserve(n_measurements);

    float range;
    for (int i=0; i<n_measurements; i++)
    {
        range = msg.ranges[index-(n_measurements/2)];
        if (range < 0.001 || range > msg.range_max)
        {
            range = MAX_RANGE;
        }
        measurements[i] = range;
    }
    return median(measurements);
}

/**
 * Returns the median of a vector of float values. Note that this function modifies the vector parameter in the calling
 * scope as it is passed by reference.
 *
 * @param distances The vector of float values to compute the median of.
 * @return The median of the provided values.
 */
float MotionPlanner::median(std::vector<float>& distances) const
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

/**
 * Returns the closest distance measurement from the LIDAR within a specified angle range.
 *
 * @param msg LaserScan msg received in the laserCallback callback function.
 * @param angle_range Vector of floats to use as the angle interval.
 * @param min_index_result Pointer to an integer where the index corresponding to the minimum distance is found.
 * @return The minimum distance measurement.
 */
float MotionPlanner::getMinRange(const sensor_msgs::LaserScan& msg, const std::vector<int>& angle_range,
    int* min_index_result=NULL) const
{
    int start_angle = angle_range[0];
    int end_angle = angle_range[1];

    // Ensure the end angle is larger than the start angle
    while (start_angle > end_angle)
    {
        end_angle += 360;
    }

    // Variables to store the minimum range found and the index it was found at
    float min_range = MAX_RANGE + 1;
    int min_index = -1;

    float range;
    for (int i=start_angle; i<=end_angle; i++)
    {
        // Get the range at the current angle
        range = getRange(msg, i % 360);

        // If a new minimum value is found, perform an update
        if (range < min_range)
        {
            min_range = range;
            min_index = i;
        }
    }

    // If a pointer was provided for min_index_result, store the minimum index value there
    if (min_index_result != NULL)
    {
        *min_index_result = min_index;
    }

    return min_range;
}

/**
 * Given a value, an interval, and a new interval, computes the new value which divides the new interval in the same
 * ratio that the given value divides the given interval. This is used in the proportional controller to map a range of
 * error values to a range of velocities.
 *
 * e.g. If a value of 0.1 is given with an interval of [0,1] and the new interval is given as [0,10], the function will
 * return the new value of 1.
 *
 * @param value Float value in the given interval.
 * @param range Vector of floats to use as the interval.
 * @param new_range Vector of floats to use as the new interval.
 * @param saturate If set to true, the return value is clipped if the value is mapped to outside the new interval.
 * @return The corresponding new value in the new interval.
 */
float MotionPlanner::mapToRange(float value, const std::vector<float>& range, const std::vector<float>& new_range,
    bool saturate=true) const
{
    float mapped_value;

    // Clip the value if it is below the range and the saturate flag is set
    if (value < range[0] && saturate)
    {
        mapped_value = new_range[0];
    }
    // Clip the value if it is above the range and the saturate flag is set
    else if (value > range[1] && saturate)
    {
        mapped_value = new_range[1];
    }
    // Calculate the new value
    else
    {
        float percent = (value - range[0]) / (range[1] - range[0]);
        mapped_value = new_range[0] + percent * (new_range[1] - new_range[0]);
    }
    return mapped_value;
}

/**
 * Controls the robot motion to make it follow a nearby left wall. This function implements a proportional controller
 * on two variables: the distance to tracked wall, and the difference in the distances measured at the front left and
 * front back of the robot. This second variable relates closely to the derivative of the first variable and hence the
 * slope/curvature of the wall. As a result, the controller should behave similarly to a proportional derivative
 * controller implemented on a single variable by achieving the specified goal with minimal overshoot.
 *
 * @param msg LaserScan msg received in the laserCallback callback function.
 */
void MotionPlanner::robotFollowWall(const sensor_msgs::LaserScan& msg) const
{
    // Velocities to publish
    float lin_vel;
    float ang_vel;

    // Get the minimum distance on the left side of the robot
    float left_min_range = getMinRange(msg, LEFT_RANGE);

    // If there is no left wall within range to track, keep moving forwards
    if (left_min_range > WALL_MAX_TRACK_DIST)
    {
        publishVelocity(MAX_LIN_VEL, 0);
        return;
    }

    // If an object is detected directly ahead, stop and turn right
    if (getMinRange(msg, FRONT_RANGE) < FRONT_TURN_DIST)
    {
        publishVelocity(0, -MAX_ANG_VEL);
        return;
    }

    // Find the error between the wall distance and the desired wall distance
    float dist_error = left_min_range - WALL_DIST;

    // Find the distance difference between the left front and left back measurements
    float angle_error = getMinRange(msg, LEFT_FRONT_RANGE) - getMinRange(msg, LEFT_BACK_RANGE);

    // Map the dist_error to an angular velocity.
    float dist_error_ang_vel = mapToRange(dist_error, DIST_ERROR_RANGE, ANG_VEL_RANGE);

    // Map the angle_error to an angular velocity.
    float angle_error_ang_vel = mapToRange(angle_error, ANGLE_ERROR_RANGE, ANG_VEL_RANGE);

    // Use the sum of the two calculated angular velocities as the final angular velocity
    ang_vel = dist_error_ang_vel + angle_error_ang_vel;

    // If the magnitude of the angular velocity is greater than the maximum, clip the value
    if (ang_vel > MAX_ANG_VEL)
    {
        ang_vel = MAX_ANG_VEL;
    }
    else if (ang_vel < -MAX_ANG_VEL)
    {
        ang_vel = -MAX_ANG_VEL;
    }

    // Set linear velocity to the maximum
    lin_vel = MAX_LIN_VEL;

    // Publish the velocity
    publishVelocity(lin_vel, ang_vel);
}

/**
 * Controls the robot motion to make it perform a scan of its surroundings by rotating once.
 *
 * @param msg LaserScan msg received in the laserCallback callback function.
 */
void MotionPlanner::robotRotate(const sensor_msgs::LaserScan& msg)
{
    // Find the angle to the wall closest to the robot
    int min_index;
    const std::vector<int> full_range = {0, 359};
    getMinRange(msg, full_range, &min_index);

    // If scan is complete and the closest wall is in LEFT_RANGE
    const int right_range_start = 250;
    const int right_range_end = 290;
    const int left_range_start = 70;
    const int left_range_end = 110;

    // If the minimum range is on the right side, then the robot has done a half turn
    if (scan_progress_ == 0 && min_index >= right_range_start && min_index <= right_range_end)
    {
        ROS_INFO("Half turn complete");
        scan_progress_ = 1;
    }
    // If the minimum range is on the left side and the robot already completed the half turn, the the robot has
    // finished the full turn
    else if (scan_progress_ == 1 && min_index >= left_range_start && min_index <= left_range_end)
    {
        ROS_INFO("Full turn complete");

        // Change robot state back to stopped
        robot_state_ = RobotState::STOPPED;
        publishScanComplete();
    }
    else
    {
        // Keep spinning clockwise
        publishVelocity(0, -MAX_ANG_VEL);
    }
}

/**
 * Callback function for the LaserScan msg which contains the LIDAR measurements. This function determines the
 * appropriate action for the robot based on the current value of the robot_state variable.
 *
 * @param msg LaserScan msg received in the laserCallback callback function.
 */
void MotionPlanner::laserCallback(const sensor_msgs::LaserScan& msg)
{
    switch (robot_state_)
    {
        case RobotState::WALL_FOLLOWING:
            robotFollowWall(msg);
            break;

        case RobotState::STOPPED:
            publishVelocity(0, 0);
            break;

        case RobotState::TAG_SCANNING:
            robotRotate(msg);
            break;

        default:
            break;
    }
}

/**
 * Callback function to update the state of the robot.
 *
 * @param msg The integer value of the state to change to.
 */
void MotionPlanner::robotStateCallback(const std_msgs::Int8& msg)
{
    // Update the robot state
    robot_state_ = msg.data;

    // If the state is being set to scanning mode, reset the scan progress to 0
    if (robot_state_ == RobotState::TAG_SCANNING)
    {
        scan_progress_ = 0;
    }
}


int main(int argc, char **argv)
{
    // Initialise the node
    ros::init(argc, argv, "motion_planner");

    // Create a node handle to pass to the class constructor
    ros::NodeHandle nh;

    // Instantiate the class
    MotionPlanner motion_planner = MotionPlanner(&nh);

    // Wait for and process callbacks
    ros::spin();

    return 0;
}

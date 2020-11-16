/**
 * map_transform.cpp
 *
 * This node continuosly publishes the PoseStamped message which contains the pose of the robot in the global map frame.
 *
 * Authors: Annie Sun, Bharath Santosh, Bohan Zhang, David Young
 */

#include "turtlebot3_disinfection/map_transform.h"

/**
 * Class constructor which initialises the pose publisher.
 *
 * @param nh ROS NodeHandle object used to instantiate this class as a ROS node.
 * @param tf_buffer Buffer object from the tf2_ros package to store transforms.
 */
MapTransform::MapTransform(ros::NodeHandle* nh, const tf2_ros::Buffer& tf_buffer)
    : tf_buffer_(tf_buffer)
{
    pub_pose_ = nh->advertise<geometry_msgs::PoseStamped>("/map_pose", 10);
}

/**
 * Computes the transform from map to base_link and publishes it on the /map_pose topic.
 */
void MapTransform::publishPose() const
{
    geometry_msgs::TransformStamped transform_stamped;

    // Attempt to look up the transform
    try {
        transform_stamped = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0)); // get most recent transform
    }
    // Print warning and exit function if the transform cannot be found
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }

    // PoseStamped message to send
    geometry_msgs::PoseStamped msg;

    // Indicate that the pose exists in the map frame
    msg.header.frame_id = "map";

    // Copy over information from the TransformStamped msg to the PoseStamped msg
    msg.pose.position.x = transform_stamped.transform.translation.x;
    msg.pose.position.y = transform_stamped.transform.translation.y;
    msg.pose.position.z = transform_stamped.transform.translation.z;
    msg.pose.orientation.x = transform_stamped.transform.rotation.x;
    msg.pose.orientation.y = transform_stamped.transform.rotation.y;
    msg.pose.orientation.z = transform_stamped.transform.rotation.z;
    msg.pose.orientation.w = transform_stamped.transform.rotation.w;

    // Publish the message
    pub_pose_.publish(msg);
}


int main(int argc, char** argv)
{
    // Initialise the node
    ros::init(argc, argv, "map_transform");

    // Create a node handle to pass to the class constructor
    ros::NodeHandle nh;

    // Create the Buffer and TransformListener objects
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Instantiate the class
    MapTransform map_transform = MapTransform(&nh, tf_buffer);

    // Set the loop rate for the publisher
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        map_transform.publishPose();
        loop_rate.sleep();
    }

    return 0;
}

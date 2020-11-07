/**
 * ... text ...
 */

#include "turtlebot3_disinfection/map_transform.h"

// Finds the current map coordinates of the robot by computing the frame transform and converts the map coordinates
// into coordinates of the occupancy grid.


MapTransform::MapTransform(ros::NodeHandle* nh)
{
    tf_listener_ = TransformListener(tf_buffer_);

    pub_pose_ = nh->advertise<geometry_msgs::PoseStamped>("/map_pose", 10);
}


int MapTransform::publishPose()
{
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0)); // get most recent transform
    }
    catch (tf2::TransformException& ex) // these happen until broadcaster starts
    {
        ROS_WARN("%s", ex.what());
        // ros::Duration(1.0).sleep();
        return 1;
    }

    geometry_msgs::PoseStamped msg;

    msg.pose.position.x = transform_stamped.transform.translation.x;
    msg.pose.position.y = transform_stamped.transform.translation.y;
    msg.pose.position.z = transform_stamped.transform.translation.z;

    msg.pose.orientation.x = transform_stamped.transform.rotation.x;
    msg.pose.orientation.y = transform_stamped.transform.rotation.y;
    msg.pose.orientation.z = transform_stamped.transform.rotation.z;
    msg.pose.orientation.w = transform_stamped.transform.rotation.w;

    pub_pose_.publish(msg);

    return 0;
}


int main()
{
    ros::init(argc, argv, "map_transform"); // Register the node on ROS

    ros::NodeHandle nh; // Create a node handle to pass to the class constructor

    MapTransform map_transform = MapTransform(&nh);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        map_transform.publishPose();
        loop_rate.sleep();
    }

    return 0;
}

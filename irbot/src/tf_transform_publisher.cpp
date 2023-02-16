#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"

// Topic messages callback
void odomTransform(const nav_msgs::Odometry &odom)
{
    //Broadcasting a transformation between "odom" frame and "base_footprint" frame
    tf::TransformBroadcaster odom_br;
    tf::Transform odom_trans;
    odom_trans.setOrigin(
        tf::Vector3(odom.pose.pose.position.x,odom.pose.pose.position.y,0.0));
    odom_trans.setRotation(
        tf::Quaternion(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,odom.pose.pose.orientation.w));
    odom_br.sendTransform(
        tf::StampedTransform(odom_trans,ros::Time::now(),"odom","base_footprint"));
}

int main (int argc, char **argv) {
    // Initiate a new ROS node named "tf_transform_publisher"
	ros::init(argc, argv, "tf_transform_publisher");
    //create a node handle: it is reference assigned to a new node
	ros::NodeHandle nh;

	//odomTransform: is the name of the callback function that will be executed each time a message is received.
    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odomTransform);

    //Broadcasting a transformation between "base_footprint" frame and "base_link" frame
    tf::TransformBroadcaster base_link_br;
    base_link_br.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0,0,0,0),tf::Vector3(0.0,0.0,0.0)),
            ros::Time::now(),"base_footprint","base_link"));

    // Enter a loop, pumping callbacks
    ros::spin();
    return 0;
}

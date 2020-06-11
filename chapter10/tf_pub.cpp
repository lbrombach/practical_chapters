/*
*tf_pub.cpp is a simple ROS node that subscribes to an odometry message
*and publishes an odom to base_link transform based on that data.
*Unlike some of node and messages we use for examples in Practical Robotics in C++,
*tf_pub uses quaternions and illustrates how to conver euler to quaternion
*This program is an accompaniment to the book Practical Robotics in C++
*written by Lloyd Brombach and published by Packt Publishing
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"


using namespace std;

//callback function that broadcasts odom message data as a transform
//this odom message should contain an euler angle in the orientation z data member
void handle_odom(const nav_msgs::Odometry &odom)
{
    //create broadcater object
    static tf::TransformBroadcaster br;
    //create transform object
    tf::Transform odom_base_tf;

    //set transform x, y location with data from odom message
    odom_base_tf.setOrigin( tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0.0) );
    //create quaternion object from euler data received in odom
    tf::Quaternion tf_q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

    //add quaternion data to transform object
    odom_base_tf.setRotation(tf_q);

    //broadcast transform
    br.sendTransform(tf::StampedTransform(odom_base_tf, odom.header.stamp, "odom", "base_link"));
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_pub");
    ros::NodeHandle node;

    ros::Subscriber subOdom = node.subscribe("encoder/odom_quat", 10, handle_odom);

    ros::Rate loop_rate(30);
    while (ros::ok)
    {
     ros::spinOnce();
     loop_rate.sleep();
    }

    return 0;
}

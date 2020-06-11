/*
*tf_echo2.cpp is a simple ROS node that displays odom to base_link 
*transform data based on a lookupTransform() call.
*Intent is a simple utility that helps teach converting from quaternion to euler angle.
*This program is an accompaniment to the book Practical Robotics in C++
*written by Lloyd Brombach and published by Packt Publishing
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float32.h"

using namespace std;

float z = 0;

int main(int argc, char **argv)
{
    //handshake with ros master
    ros::init(argc, argv, "tf_echo2");
    ros::NodeHandle node;

    //set to 1 hz
    ros::Rate loop_rate(1);
    while (ros::ok)
    {
        ros::spinOnce();
        //create listener. Must be static.
        static tf::TransformListener listener;

        //create transform object
        tf::StampedTransform odom_base_tf;

        //test if transform available
        if (listener.canTransform("odom", "base_link", ros::Time(0), NULL))
        {
            //get transform data
            listener.lookupTransform("odom", "base_link", ros::Time(0), odom_base_tf);

            //convert quaternion data to Euler angle
            tf::Quaternion q(0, 0, odom_base_tf.getRotation().z(), odom_base_tf.getRotation().w());
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            z = yaw;

            //display transform data
            cout << odom_base_tf.getOrigin().x() << ", " << odom_base_tf.getOrigin().y() << "   " << z << endl;
        }
        else
        {
            cout << "UNABLE TO LOOKUP ODOM -> BASE_LINK TRANSFORM" << endl;
        }
        loop_rate.sleep();
    }

    return 0;
}

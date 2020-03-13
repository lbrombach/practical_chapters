/*
*tf_pub.cpp is a simple ROS node that subscribes to an odometry message
*and published an odom to base_link transform based on that data.
*Unlike some of node and messages we use for examples in Practical Robotics in C++,
*tf_pub uses quaternions
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
//#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float32.h"


using namespace std;


float z = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_echo2");
    ros::NodeHandle node;


    ros::Rate loop_rate(1);
    while (ros::ok)
    {
     ros::spinOnce();



         static tf::TransformListener listener;
    tf::StampedTransform odom_base_tf;

    if(listener.canTransform("odom","base_link", ros::Time(0), NULL))
    {
        listener.lookupTransform("odom", "base_link", ros::Time(0), odom_base_tf);

        //dont forget that grid cell is pose in meters / map resolution
       // start.x = odom_base_tf.getOrigin().x()/ map_resolution(_map);
       // start.y = odom_base_tf.getOrigin().y()/ map_resolution(_map);

        tf::Quaternion q(0, 0, odom_base_tf.getRotation().z(), odom_base_tf.getRotation().w());
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        z = yaw;
    cout<<odom_base_tf.getOrigin().x()<<", "<<odom_base_tf.getOrigin().y()<<"   "<<z<<endl;
    }
    else
    {
        cout<<"UNABLE TO LOOKUP ODOM -> BASE_LINK TRANSFORM, no path planned"<<endl;
    }
     loop_rate.sleep();
    }


    return 0;
}

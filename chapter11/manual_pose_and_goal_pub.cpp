/*
*manual_pose_and_goal.cpp
*
*This ros node publishes an initial_2d pose msg or goal_2d pose message as input by the user.
*these msgs are published ignoring the quaternion nature and simply using z for yaw
*the initialpose and move_base_simple/goal are also published as if user had clicked in rviz
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/4/2019
*/
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>

using namespace std;
ros::Publisher pub;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;


void pub_msg(float x, float y, float yaw, int choice)
{
    geometry_msgs::PoseWithCovarianceStamped pose;
    geometry_msgs::PoseStamped goal;
    geometry_msgs::PoseStamped rpy;
    rpy.pose.position.x = x;
    rpy.pose.position.y = y;
    rpy.pose.position.z = 0;
    rpy.pose.orientation.x = 0;
    rpy.pose.orientation.y = 0;
    rpy.pose.orientation.z = yaw;
    rpy.pose.orientation.w = 0;

    if(choice == 1) //publish a pose
    {
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.pose = rpy.pose;
        pub2.publish(rpy);
    }
    else //publish a goal
    {
        cout<<"publishering GOAL"<<endl;

        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose = rpy.pose;
        pub.publish(goal);
    }

     tf2::Quaternion q;
     q.setRPY(0, 0, yaw);
        rpy.pose.orientation.x = q.x();
        rpy.pose.orientation.y = q.y();
        rpy.pose.orientation.z = q.z();
        rpy.pose.orientation.w = q.w();

        if(choice == 1) //publish a pose
    {
        pose.pose.pose = rpy.pose;
        pub3.publish(pose);
    }
    else //publish a goal
    {
        goal.pose = rpy.pose;
        pub1.publish(goal);
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_pub");
    ros::NodeHandle node;
    pub = node.advertise<geometry_msgs::PoseStamped>("goal_2d", 10);
    pub1 = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
    pub2 = node.advertise<geometry_msgs::PoseStamped>("initial_2d", 10);
    pub3 = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);

    while (ros::ok())
    {
        float x =-1;
        float y =-1;
        float yaw = -11;
        int choice  = -1;

        while(choice < 1 || choice > 2
            || x < 0 || y < 0 || yaw < -3.141592 || yaw > 3.141592)
        {
            cout<<"\nEnter positive float value for x : "<<endl;
            cin>>x;
            cout<<"\nEnter positive float value for y : "<<endl;
            cin>>y;
            cout<<"\nEnter float value in for yaw in radians from -PI to +PI: "<<endl;
            cin>>yaw;


            cout<<"\nEnter 1 if this is a pose, 2 if it is a goal"<<endl;
            cin>>choice;
            if(cin.fail())
            {
                cin.clear();
                cin.ignore(50, '/n');
                choice = -1;
            }
        }

        pub_msg(x, y, yaw, choice);

    }
    return 0;
}

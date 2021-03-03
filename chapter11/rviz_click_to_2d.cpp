/*
*rviz_click_to_2d.cpp
*
*This ros node subscribes to the clicked pose topics published by rviz
*initialpose and move_base_simple/goal, and republishes the data as simple
*2D poses using PoseStamped messages, after converting from quaternion to euler angles
*Intended to accompany example robot/nodes along with the book Practical Robotics in C++
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/4/2019
*/
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
using namespace std;
ros::Publisher pub;
ros::Publisher pub2;

void handle_goal(const geometry_msgs::PoseStamped &goal)
{
    geometry_msgs::PoseStamped rpyGoal;
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp = goal.header.stamp;
    rpyGoal.pose.position.x = goal.pose.position.x;
    rpyGoal.pose.position.y = goal.pose.position.y;
    rpyGoal.pose.position.z = 0;
    tf::Quaternion q(0, 0, goal.pose.orientation.z, goal.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    rpyGoal.pose.orientation.x = 0;
    rpyGoal.pose.orientation.y = 0;
    rpyGoal.pose.orientation.z = yaw;
    rpyGoal.pose.orientation.w = 0;
    pub.publish(rpyGoal);
}
void handle_initial_pose(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    geometry_msgs::PoseStamped rpyPose;
    rpyPose.header.frame_id = "map";
    rpyPose.header.stamp = pose.header.stamp;
    rpyPose.pose.position.x = pose.pose.pose.position.x;
    rpyPose.pose.position.y = pose.pose.pose.position.y;
    rpyPose.pose.position.z = 0;
    tf::Quaternion q(0, 0, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    rpyPose.pose.orientation.x = 0;
    rpyPose.pose.orientation.y = 0;
    rpyPose.pose.orientation.z = yaw;
    rpyPose.pose.orientation.w = 0;
    pub2.publish(rpyPose);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rviz_click_to_2d");
    ros::NodeHandle node;
    pub = node.advertise<geometry_msgs::PoseStamped>("goal_2d", 0);
    pub2 = node.advertise<geometry_msgs::PoseStamped>("initial_2d", 0);
    ros::Subscriber sub = node.subscribe("move_base_simple/goal", 0, handle_goal);

    ros::Subscriber sub2 = node.subscribe("initialpose", 0, handle_initial_pose);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

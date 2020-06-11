/*
simplegoal_pub.cpp is a program to learn to write ROS publisher nodes.
This node publishes the Pose2D data type required by the go_to_xy.cpp program
that drives the turtlesim turtle to the location entered here. 
This program is an accompaniment to the book Practical Robotics in C++
written by Lloyd Brombach and published by Packt Publishing
*/

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <iostream>

geometry_msgs::Pose2D goal;

using namespace std;

void set_goal()
{
    cout << "enter goal x: ";
    cin >> goal.x;
    cout << goal.x << " now y: " << endl;
    cin >> goal.y;
    cout << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_goal_pub");
    ros::NodeHandle node;

    ros::Publisher pubGoal = node.advertise<geometry_msgs::Pose2D>("waypoint", 0);

    while (ros::ok)
    {

        set_goal();
        pubGoal.publish(goal);
        cout << "publishing" << endl;
   
    }

    return 0;
}
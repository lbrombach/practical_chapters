/*
go_to_x.cpp is a program to learn to write ROS nodes.
This node commands the turtlesim turtle to the X location
manually entered in the const double GOAL below.
Change the goal to any value from 0 to 11 and recompile with catkin_make before running.
This program is an accompaniment to the book Practical Robotics in C++
written by Lloyd Brombach and published by Packt Publishing
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "turtlesim/Pose.h"
#include <cstdlib> //for abs()
#include <iostream>

using namespace std;

//declaring variables. 
geometry_msgs::Twist cmdVel;
geometry_msgs::Pose2D current;
geometry_msgs::Pose2D desired;

//change GOAL to any value from 0 to 11
const double GOAL = 1.5;

//the coefficient (or gain) for our linear velocity calculation
const double Kl = 1;

//the distance we are willing to accept as “close enough”
const double distanceTolerance = .1;

void misc_setup()
{
    desired.x = GOAL;
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.linear.z = 0;
}

// callback function to update the current location 
//we will use later. We don’t use the y or theta data members
//in this tutorial, but I included them so you could see how
//to access them in the future.
void update_pose(const turtlesim::PoseConstPtr &currentPose)
{
    current.x = currentPose->x;
    current.y = currentPose->y;
    current.theta = currentPose->theta;
}

double getDistanceError()
{
    return desired.x - current.x;
}

//if we aren’t close enough, set the cmd_vel data member
//to be published later. Else set it to zero.
void set_velocity()
{
    if (abs(getDistanceError()) > distanceTolerance)
    {
        cmdVel.linear.x = Kl * getDistanceError();
    }
    else
    {
        cout << "I'm HERE!" << endl;
        cmdVel.linear.x = 0;
    }
}

int main(int argc, char **argv)
{
    misc_setup();  

    //register node “go_to_x” with roscore, then get a nodehandle
    ros::init(argc, argv, "go_to_x");
    ros::NodeHandle node;

    //Subscribe to topic and set callback **See detail in figure 9.10
    ros::Subscriber subCurrentPose =
	node.subscribe("turtle1/pose", 0, update_pose);

    //Register node as publisher **See detail in figure 9.10
    ros::Publisher pubVelocity =
      node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);

    //set the frequency that you want the loop below to execute at
    ros::Rate loop_rate(10); //10 cycles per second

    //execute this loop until connection is lost with ROS Master 
    while (ros::ok)
    {
        //call the callbacks waiting to be called.
        ros::spinOnce();

        //call our own function after the callbacks are done
        set_velocity();

        //publish the message that we internally called cmdVel
        pubVelocity.publish(cmdVel);

        //output for you entertainment
        cout << "goal x = " << desired.x << endl
             << "current x = " << current.x <<  endl
             << "  disError = " << getDistanceError() << endl
             << "cmd_vel = " << cmdVel.linear.x<< endl;

        //We set the frequency for 10Hz, this sleeps as long as it 
        //takes to keep that frequency
        loop_rate.sleep();
    }

    return 0;
}
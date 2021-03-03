/*
go_to_xy.cpp is a program to learn to write ROS nodes.
This node commands the turtlesim turtle to the X location
manually entered in the accompanying program simple_goal_pub.cpp.
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
ros::Publisher pubVelocity;
geometry_msgs::Twist cmdVel;
geometry_msgs::Pose2D current;
geometry_msgs::Pose2D desired;
const double PI = 3.141592;

//the coefficient (or gain) for our angular velocity calculation
const double Ka = .5;   

//the coefficient (or gain) for our linear velocity calculation
const double Klv = .5;

//the angle we'll call "close enough"
const double angularTolerance = .1;

//the distance we are willing to accept as “close enough”
const double distanceTolerance = .1;

//a flag that we use to to start and stop calculating and publishing commands
bool waypointActive = false;

void misc_setup()
{
    desired.x = 5.54;  //this is just where turtle starts
    desired.y = 5.54;  //this is just where turtle starts
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.linear.z = 0;
}

// callback function to update the current location 
void update_pose(const turtlesim::PoseConstPtr &currentPose)
{
    current.x = currentPose->x;
    current.y = currentPose->y;
    current.theta = currentPose->theta;
}

// callback function to update goal when goal message received
void update_goal(const geometry_msgs::Pose2D &desiredPose)
{
    desired.x = desiredPose.x;
    desired.y = desiredPose.y;
    waypointActive = true;
}


// calculates straight-line distance to goal
double getDistanceError()
{
    return sqrt(pow(desired.x - current.x, 2) + pow(desired.y - current.y, 2));
}

// calculates error in heading angle
double getAngularError()
{
    double deltaX = desired.x - current.x;
    double deltaY = desired.y - current.y;
    double thetaBearing = atan2(deltaY, deltaX);
    double angularError = thetaBearing - current.theta;
    angularError = (angularError > PI) ? angularError - (2 * PI) : angularError;
    angularError = (angularError < -PI) ? angularError + (2 * PI) : angularError;
    return angularError;
}

void set_velocity()
{
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.linear.z = 0;

    double angularError = getAngularError();
    double distanceError = getDistanceError();

    if (waypointActive == true && abs(distanceError) > .15)
    {
        if (abs(angularError) < .1)
        {
            cmdVel.linear.x = Klv * distanceError;
            cmdVel.angular.z = 0;
        }
        else
        {
            cmdVel.angular.z = Ka * angularError;
            cmdVel.linear.x = 0;
        }
    }
    else
    {
        cout << "I'm HERE!" << endl;
        cmdVel.linear.x = 0;
        cmdVel.angular.z = 0;
        waypointActive = false;
    }

    pubVelocity.publish(cmdVel);
}

int main(int argc, char **argv)
{
    misc_setup();  

    //register node “go_to_x” with roscore, then get a nodehandle
    ros::init(argc, argv, "go_to_x");
    ros::NodeHandle node;

    //Subscribe to current pose topic and set callback to "update_pose"
    ros::Subscriber subCurrentPose =
	    node.subscribe("turtle1/pose", 0, update_pose);

    //subscribe to goal location topic "waypoint" and set callback to "update_goal"
    ros::Subscriber subDesiredPose = node.subscribe("waypoint", 0, update_goal, ros::TransportHints().tcpNoDelay());
    pubVelocity = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

    //Register node as publisher of type geometry_msgs::Twist and topic name turtle1/cmd_vel
    ros::Publisher pubVelocity =
      node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 0);

    //set the frequency that you want the loop below to execute at
    ros::Rate loop_rate(10); //10 cycles per second

    //execute this loop until connection is lost with ROS Master 
    while (ros::ok())
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
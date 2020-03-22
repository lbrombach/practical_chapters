/*
*simple_drive_controller.cpp is an example ROS node accompanying the book
*Practical Robotics in C++.
*
*This node subscribes to the encoder/odom topic as well as waypoint_2d. When a new
*waypoint is recieved, this node calculates a straight course to the waypoint without obstacle avoidance
*and publishes cmd_vel msgs to first turn to the waypoint, then go forward. If the angle drifts
*outside of "close enough" while enroute, the robot will stop and correct is heading before continuing.
*Written to be a readable, but functional, example for all levels following along with the book.
*
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <cstdlib>
#include <math.h>
#include <iostream>

using namespace std;

ros::Publisher pubVelocity;
nav_msgs::Odometry odom;
geometry_msgs::Twist cmdVel;
geometry_msgs::PoseStamped desired;
const double PI = 3.141592;
const double Ka = .35;
const double Klv = .66;
const double initialX = 5.0;
const double initialY = 5.0;
const double angularTolerance = .1;
const double distanceTolerance = .05;
const double MAX_LINEAR_VEL = 1;
bool waypointActive = false;


void update_pose(const nav_msgs::Odometry &currentOdom)
{
    odom.pose.pose.position.x = currentOdom.pose.pose.position.x;
    odom.pose.pose.position.y = currentOdom.pose.pose.position.y;
    odom.pose.pose.orientation.z = currentOdom.pose.pose.orientation.z;
}

void update_goal(const geometry_msgs::PoseStamped &desiredPose)
{
cout<<"got new goal!"<<endl;
    desired.pose.position.x = desiredPose.pose.position.x;
    desired.pose.position.y = desiredPose.pose.position.y;
    desired.pose.orientation.z = desiredPose.pose.orientation.z;
    waypointActive = true;
    cout<<"waypoint active set true"<<endl;
}

double getDistanceError()
{
    double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
    double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
    cout<<"Distance error = "<<sqrt(pow(deltaX, 2) + pow(deltaY, 2))<<endl;
    return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

double getAngularError()
{
    double deltaX = desired.pose.position.x - odom.pose.pose.position.x;
    double deltaY = desired.pose.position.y - odom.pose.pose.position.y;
    double thetaBearing = atan2(deltaY, deltaX);
    double angularError = thetaBearing - odom.pose.pose.orientation.z;
    angularError = (angularError > PI)  ? angularError - (2*PI) : angularError;
    angularError = (angularError < -PI) ? angularError + (2*PI) : angularError;
cout<<"angular error = " <<angularError<<endl;
    return angularError;
}


void set_velocity()
{
    cmdVel.linear.x = 0;
    cmdVel.linear.y = 0;
    cmdVel.linear.z = 0;
    cmdVel.angular.x = 0;
    cmdVel.angular.y = 0;
    cmdVel.angular.z = 0;

    static bool angle_met = true;
    static bool location_met = true;
    double final_desired_heading_error = desired.pose.orientation.z - odom.pose.pose.orientation.z;

    if(abs(getDistanceError()) >= .05)
        {
        location_met = false;
        }
    else if (abs(getDistanceError()) < .03)
        {
        location_met = true;
        }

    double angularError = (location_met == false) ? getAngularError() : final_desired_heading_error;
    if (abs(angularError) > .15)
        {
         angle_met = false;
        }
    else if (abs(angularError) < .1)
        {
         angle_met = true;
        }

    if (waypointActive == true && angle_met == false)
        {
         cmdVel.angular.z = Ka * angularError;
         cmdVel.linear.x = 0;
        }
    else if (waypointActive == true && abs(getDistanceError()) >= .05 && location_met == false)
        {
         cmdVel.linear.x = Klv * getDistanceError();
         cmdVel.angular.z = 0;
        }
    else
    {
        cout << "********I'm HERE, now set final desired heading! **********"<<endl;
        location_met = true;
    }

    if (location_met && abs(final_desired_heading_error) < .05)
        {
         cout<<"Target Achieved"<<endl;
         waypointActive = false;
        }

    pubVelocity.publish(cmdVel);

}


int main(int argc, char **argv)
{
    desired.pose.position.x = -1;
    ros::init(argc, argv, "simple_drive_controller");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subCurrentPose = node.subscribe("encoder/odom", 10, update_pose, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subDesiredPose = node.subscribe("waypoint_2d", 1, update_goal, ros::TransportHints().tcpNoDelay());
    pubVelocity = node.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate loop_rate(10);
    while (ros::ok)
    {
        ros::spinOnce();
        if(desired.pose.position.x != -1)
        {
        set_velocity();
        }
        cout << "goal = " << desired.pose.position.x << ", " << desired.pose.position.y << endl
             << "current x,y = " << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << endl
             << "  Distance error = " << getDistanceError() << endl;
        cout << "cmd_vel = " << cmdVel.linear.x <<" ,  "<<cmdVel.angular.z<< endl;
        loop_rate.sleep();
    }

    return 0;
}

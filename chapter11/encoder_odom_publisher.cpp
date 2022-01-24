/*
*This is a ROS node that subscribes to encoder count messages and publishes odometry
*on in a simple form where orientation.z is an euler angle, then publishes again
*on a topic that uses the quaternion version of the message. This is written
*to be readable for all levels and accompanies the book Practical Robotics in C++.
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

ros::Publisher odom_pub;
ros::Publisher pub_quat;
nav_msgs::Odometry newOdom;
nav_msgs::Odometry oldOdom;

const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

const double ticksPerwheelRev = 508.8; //not in use. just a reference for now
const double wheelRadius = .03575; // 55.18;
const double WHEEL_BASE = .224; //223.8375mm actually
const double TICKS_PER_M = 1125*2; //1.1645; //1.365 is on hard floor. carpet avg is 1.1926. overall avg = 1.1645

double leftDistance = 0;
double rightDistance = 0;
bool initialPoseRecieved = false;


using namespace std;

//accept initial_2d pose message form clicks or manual publisher
//at at least one is required to start this node publishing
void setInitialPose(const geometry_msgs::PoseStamped &rvizClick)
{
     oldOdom.pose.pose.position.x = rvizClick.pose.position.x;
     oldOdom.pose.pose.position.y = rvizClick.pose.position.y;
     oldOdom.pose.pose.orientation.z = rvizClick.pose.orientation.z;
     initialPoseRecieved = true;
}

//calculate distance left wheel has traveled since last cycle
void Calc_Left(const std_msgs::Int16& lCount)
{
static int lastCountL = 0;
    if(lCount.data!=0&&lastCountL!=0)
    {
        int leftTicks = (lCount.data - lastCountL);

        if (leftTicks > 10000)
        {
            leftTicks=0-(65535 - leftTicks);
        }
        else if (leftTicks < -10000)
        {
            leftTicks = 65535-leftTicks;
        }
        leftDistance = leftTicks/TICKS_PER_M;
    }
        lastCountL = lCount.data;
}

//calculate distance right wheel has traveled since last cycle
void Calc_Right(const std_msgs::Int16& rCount)
{
static int lastCountR = 0;
    if(rCount.data!=0&&lastCountR!=0)
    {
        int rightTicks = rCount.data - lastCountR;
        if (rightTicks > 10000)
        {
            rightTicks=(0-(65535 - rightTicks))/TICKS_PER_M;
        }
        else if (rightTicks < -10000)
        {
        rightTicks = 65535 - rightTicks;
        }
        rightDistance = rightTicks/TICKS_PER_M;
    }
        lastCountR=rCount.data;
}

//publishes the simpler version of the odom message in full quaternion form
//and includes covariance data. This covariance should be "personalized"
//for every specific robot.
void publish_quat()
{
    tf2::Quaternion q;
    q.setRPY(0, 0, newOdom.pose.pose.orientation.z);

    nav_msgs::Odometry quatOdom;
    quatOdom.header.stamp = newOdom.header.stamp;
    quatOdom.header.frame_id = "odom";
    quatOdom.child_frame_id = "base_link";
    quatOdom.pose.pose.position.x = newOdom.pose.pose.position.x;
    quatOdom.pose.pose.position.y = newOdom.pose.pose.position.y;
    quatOdom.pose.pose.position.z = newOdom.pose.pose.position.z;
    quatOdom.pose.pose.orientation.x = q.x();
    quatOdom.pose.pose.orientation.y = q.y();
    quatOdom.pose.pose.orientation.z = q.z();
    quatOdom.pose.pose.orientation.w = q.w();
    quatOdom.twist.twist.linear.x = newOdom.twist.twist.linear.x;
    quatOdom.twist.twist.linear.y = newOdom.twist.twist.linear.y;
    quatOdom.twist.twist.linear.z = newOdom.twist.twist.linear.z;
    quatOdom.twist.twist.angular.x = newOdom.twist.twist.angular.x;
    quatOdom.twist.twist.angular.y = newOdom.twist.twist.angular.y;
    quatOdom.twist.twist.angular.z = newOdom.twist.twist.angular.z;

    for(int i = 0; i<36; i++)
    {
       if(i == 0 || i == 7 || i == 14)
       {
           quatOdom.pose.covariance[i] = .01;
       }
       else if (i == 21 || i == 28 || i== 35)
       {
           quatOdom.pose.covariance[i] += .165;
       }
       else
       {
           quatOdom.pose.covariance[i] = 0;
       }
    }

    pub_quat.publish(quatOdom);

}

void update_odom()
{
    //average distance
    double cycleDistance = (rightDistance+leftDistance)/2;
    //how many radians robot has turned since last cycle
    double cycleAngle = asin((rightDistance-leftDistance)/WHEEL_BASE);

    //average angle during last cycle
    double avgAngle = cycleAngle/2 + oldOdom.pose.pose.orientation.z;
    if (avgAngle > PI)
        {
        avgAngle -= 2*PI;
        }
    else if (avgAngle < -PI)
    {
        avgAngle += 2*PI;
    }

    //calculate new x, y, and theta
    newOdom.pose.pose.position.x = oldOdom.pose.pose.position.x + cos(avgAngle)*cycleDistance;
    newOdom.pose.pose.position.y = oldOdom.pose.pose.position.y + sin(avgAngle)*cycleDistance;
    newOdom.pose.pose.orientation.z = cycleAngle + oldOdom.pose.pose.orientation.z;

    //prevent lockup from a single erroneous cycle
    if(isnan(newOdom.pose.pose.position.x) || isnan(newOdom.pose.pose.position.y)
        || isnan(newOdom.pose.pose.position.z) )
    {
        newOdom.pose.pose.position.x = oldOdom.pose.pose.position.x;
        newOdom.pose.pose.position.y = oldOdom.pose.pose.position.y;
        newOdom.pose.pose.orientation.z = oldOdom.pose.pose.orientation.z;
    }

    //keep theta in range proper range
    if (newOdom.pose.pose.orientation.z > PI)
    {
        newOdom.pose.pose.orientation.z -= 2*PI;
    }
    else if (newOdom.pose.pose.orientation.z < -PI)
    {
        newOdom.pose.pose.orientation.z += 2*PI;
    }

    //calculate velocity
    newOdom.header.stamp = ros::Time::now();
    newOdom.twist.twist.linear.x = cycleDistance/(newOdom.header.stamp.toSec() - oldOdom.header.stamp.toSec());
    newOdom.twist.twist.angular.z = cycleAngle/(newOdom.header.stamp.toSec() - oldOdom.header.stamp.toSec());

    //save odom x, y, and theta for use in next cycle
    oldOdom.pose.pose.position.x = newOdom.pose.pose.position.x;
    oldOdom.pose.pose.position.y = newOdom.pose.pose.position.y;
    oldOdom.pose.pose.orientation.z = newOdom.pose.pose.orientation.z;
    oldOdom.header.stamp = newOdom.header.stamp;

    //publish odom message
    odom_pub.publish(newOdom);
}

int main(int argc, char **argv)
{
    //set fixed data fields
    newOdom.header.frame_id = "odom";
    newOdom.pose.pose.position.z = 0;
    newOdom.pose.pose.orientation.x = 0;
    newOdom.pose.pose.orientation.y = 0;
    newOdom.twist.twist.linear.x = 0;
    newOdom.twist.twist.linear.y = 0;
    newOdom.twist.twist.linear.z = 0;
    newOdom.twist.twist.angular.x = 0;
    newOdom.twist.twist.angular.y = 0;
    newOdom.twist.twist.angular.z = 0;
    oldOdom.pose.pose.position.x = initialX;
    oldOdom.pose.pose.position.y = initialY;
    oldOdom.pose.pose.orientation.z = initialTheta;

    //handshake with ros master and create node object
    ros::init(argc, argv, "encoder_odom_publisher");
    ros::NodeHandle node;

    //Subscribe to topics
    ros::Subscriber subForRightCounts = node.subscribe("rightWheel", 100, Calc_Right,ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForLeftCounts = node.subscribe("leftWheel",100, Calc_Left,ros::TransportHints().tcpNoDelay());
    ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, setInitialPose);

    //advertise publisher of simpler odom msg where orientation.z is an euler angle
    odom_pub = node.advertise<nav_msgs::Odometry>("encoder/odom", 100);

    //advertise publisher of full odom msg where orientation is quaternion
    pub_quat = node.advertise<nav_msgs::Odometry>("encoder/odom_quat", 100);


    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        if(initialPoseRecieved)
        {
            update_odom();
            publish_quat();
        }
        loop_rate.sleep();
    }

    return 0;
}

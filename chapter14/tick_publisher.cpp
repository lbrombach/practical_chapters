/*
*This is a ROS node that monitors a pair of hall effect encoders and publishes
*the tick counts for a left wheel and right wheel in ROS. Whether each
*GPIO event is incremented or decremented is determined by check the direction
*signal going to the motor driver. This is written simply
*to be readable for all levels and accompanies the book Practical Robotics in C++.
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <pigpiod_if2.h>
#include <iostream>

using namespace std;

//GPIO Pin assignments
const int leftEncoder = 22; //left encoder
const int rightEncoder = 23; //right encoder
const int leftReverse = 13; //monitor as input that goes low when left motor set to reverse
const int rightReverse = 19; //monitor as input that goes low when right motor set to reverse

//max and min allowable values
const int encoderMin = -32768;
const int encoderMax = 32768;

std_msgs::Int16 leftCount;
std_msgs::Int16 rightCount;


//this is the callback function that runs when a change of state happens on the monitored gpio pin
void left_event(int pi, unsigned int gpio, unsigned int edge, unsigned int tick)
{
if(gpio_read(pi, leftReverse)==0) //decrement if motor commanded to reverse
    {
     if(leftCount.data==encoderMin) //handle rollunder
     {
      leftCount.data = encoderMax;
     }
     else
     {
      leftCount.data--;
     }

    }
else  //increment if not commanded to reverse (must be going forward)
    {
     if(leftCount.data==encoderMax) //handle rollover
     {
      leftCount.data = encoderMin;
     }
     else
     {
      leftCount.data++;
     }

    }

}
//this is the callback function that runs when a change of state happens on the monitored gpio pin
void right_event(int pi, unsigned int gpio, unsigned int edge, unsigned int tick)
{
if(gpio_read(pi, rightReverse)==0)
    {
     if(rightCount.data==encoderMin)
     {
      rightCount.data = encoderMax;
     }
     else
     {
      rightCount.data--;
     }
    }
else
    {
     if(rightCount.data==encoderMax)
      {
       rightCount.data = encoderMin;
      }
      else
      {
       rightCount.data++;
      }
    }

}

int PigpioSetup()
{
    char *addrStr = NULL;
    char *portStr = NULL;
    int pi = pigpio_start(addrStr, portStr);

    //set the mode and pullup to read the encoder like a switch
    set_mode(pi, leftEncoder, PI_INPUT);
    set_mode(pi, rightEncoder, PI_INPUT);
    set_mode(pi, leftReverse, PI_INPUT);
    set_mode(pi, rightReverse, PI_INPUT);
    set_pull_up_down(pi, leftEncoder, PI_PUD_UP);
    set_pull_up_down(pi, rightEncoder, PI_PUD_UP);
    set_pull_up_down(pi, leftReverse, PI_PUD_UP);
    set_pull_up_down(pi, rightReverse, PI_PUD_UP);

    return pi;
}

int main(int argc, char **argv)
{
    //initialize pipiod interface
    int pi = PigpioSetup();
    if(pi>=0)
    {
        cout<<"daemon interface started ok at "<<pi<<endl;
    }
    else
    {
        cout<<"Failed to connect to PIGPIO Daemon - is it running?"<<endl;
        return -1;
    }


    //initializes callbacks
    int cbLeft=callback(pi, leftEncoder,EITHER_EDGE, left_event);
    int cbRight=callback(pi, rightEncoder, EITHER_EDGE, right_event);

    //normal ROS node setup: Register node with master,  advertise publishers
    ros::init(argc, argv, "tick_publisher");
    ros::NodeHandle node;
    ros::Publisher pubLeft = node.advertise<std_msgs::Int16>("leftWheel", 1000);
    ros::Publisher pubRight = node.advertise<std_msgs::Int16>("rightWheel", 1000);


    ros::Rate loop_rate(30);
    while(ros::ok)
    {
        pubLeft.publish(leftCount);
        pubRight.publish(rightCount);
        ros::spinOnce();

        loop_rate.sleep();
    }
    //terminate callbacks and pigpiod connectoin to release daemon resources
    callback_cancel(cbLeft);
    callback_cancel(cbRight);
    pigpio_stop(pi);
    return 0;
}

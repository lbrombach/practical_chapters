/*
*This is a ROS node that reads an HC-SR04 ultrasonic range sensor
*then publishes a ROS sensor_msgs::Range message. This is written
*to be readable for all levels and accompanies the book Practical Robotics in C++.
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <pigpiod_if2.h>
#include <iostream>

//assign alias to the gpio pin numbers
const int trigger =  6;
const int echo = 16;

sensor_msgs::Range range;

using namespace std;

//Returns range in centimers
float get_range(int pi)
{
//initiate reading with 10 microsecond pulse to trigger
gpio_write(pi, trigger, 1);
time_sleep(.00001);
gpio_write(pi, trigger, 0);

//wait for echo pin to go from low to high
while(gpio_read(pi, echo)==0){};

//get current tick (microseconds since boot) from system.
int start = get_current_tick(pi);

//wait for echo pin to return to return to low
while(gpio_read(pi, echo)==1){};

//calculate duration of high pulse, which is equal to round trip time
int echoTime = get_current_tick(pi) - start;

//speed of sound is 343 m/s, but total echo time is round trip
//half that times echo time (in seconds) is the range
return 171.5 * echoTime / 1000000;
}

//connects with pigpio daemon, intializes gpio pin modes and states
int PigpioSetup()
{
  char *addrStr = NULL;
  char *portStr = NULL;
  int pi = pigpio_start(addrStr, portStr);

  //set the pin modes and set the trigger output to low
  set_mode(pi, echo, PI_INPUT);
  set_mode(pi, trigger, PI_OUTPUT);
  gpio_write(pi, trigger, 0);

  //let pins stabilize before trying to read
  time_sleep(.01);

  return pi;
}

int main(int argc, char **argv)
{
  //normal ROS node setup: Register node with master,  advertise publisher
  ros::init(argc, argv, "ultrasonic_publisher");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Range>("ultra_range", 0);

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

  //set our range message fixed data
  range.header.frame_id = "sonar";
  //The message has enums for radiation_type where 0 = ultrasonic and 1= infrared
  range.radiation_type = 0;
  range.field_of_view = .35;
  range.min_range = .02;
  range.max_range = 4.0;


  ros::Rate loop_rate(10);
  while(ros::ok)
  {
    range.header.stamp = ros::Time::now();
    range.range = get_range(pi);
    pub.publish(range);
    loop_rate.sleep();
  }

  pigpio_stop(pi);
  return 0;
}

/*
*This is a ROS node that that reads and publishes
*an LSM303DLHC accelerometer and L3GD20 gyroscope in ROS. This is written simply
*to be readable for all levels and accompanies lessons int the book Practical Robotics in C++.
*
*Author: Lloyd Brombach (lbrombach2@gmail.com)
*11/7/2019
*/

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <pigpiod_if2.h>
#include <iostream>

const int LSM303_accel=0x19; //accelerometer address
const int LSM303_mag  =0x1e; //magnetometer address
const int L3GD20_gyro = 0x6b; //gyroscope address
const int I2Cbus=1; //RPi typical I2C bus. Find yours with "ls /dev/*i2c* "
const float RAD_PER_DEG = 0.0174533;

int pi = -1;
int ACCEL_HANDLE=-1;
int GYRO_HANDLE=-1;

//create Imu message object
sensor_msgs::Imu myImu;

using namespace std;



void gyro_setup()
{
    //initiate comms with gyroscope and get handle
    GYRO_HANDLE=i2c_open(pi,I2Cbus, L3GD20_gyro,0);
    if (GYRO_HANDLE>=0)
    {
      cout<<"Gyro found. Handle = "<<GYRO_HANDLE<<endl;
    }
    else
    {
      cout<<"Unable to open I2C comms with Gyro"<<endl;
    }


    time_sleep(.05);

    i2c_write_byte_data(pi, GYRO_HANDLE, 0x20, 95); //set frequency
    time_sleep(.05);
}

void get_gyro()
{
    //readings in default scale of 245 degrees per sec
    //sensitivity at 245 deg/sec is .00875 degrees per digit
    int xLSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, 0x28);
    int xMSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, 0x29);
    myImu.angular_velocity.x=(float)((int16_t)(xLSB | xMSB<<8))*.00875*RAD_PER_DEG;

    int yLSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, 0x2A);
    int yMSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, 0x2B);
    myImu.angular_velocity.y=(float)((int16_t)(yLSB | yMSB<<8))*.00875*RAD_PER_DEG;

    int zLSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, 0x2C);
    int zMSB = (int)i2c_read_byte_data(pi, GYRO_HANDLE, 0x2D);
    myImu.angular_velocity.z=(float)((int16_t)(zLSB | zMSB<<8))*.00875*RAD_PER_DEG;
}

void accel_setup()
{
    //initiate comms with accelerometer and get handle
    ACCEL_HANDLE=i2c_open(pi,I2Cbus, LSM303_accel,0);
    if (ACCEL_HANDLE>=0)
    {
        cout<<"Accelerometer found. Handle = "<<ACCEL_HANDLE<<endl;
    }
    else
    {
        cout<<"Unable to open I2C comms with Accelerometer"<<endl;
    }

    i2c_write_byte_data(pi, ACCEL_HANDLE, 0x20, 0x47); //set frequency
    time_sleep(.02);
    i2c_write_byte_data(pi, ACCEL_HANDLE, 0x23, 0x09); //continuous update, LSB at lower addr, +- 2g, Hi-Res disable
    time_sleep(.02);
}

void get_accel()
{
    int xLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x28);
    int xMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x29);
    //12 bits resolution, MSB right-jusified, then convert to Tesla
    myImu.linear_acceleration.x=(float)((int16_t)(xLSB | xMSB<<8)>>4)/1000*9.81;

    int yLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2A);
    int yMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2B);
    myImu.linear_acceleration.y=(float)((int16_t)(yLSB | yMSB<<8)>>4)/1000*9.81;

    int zLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2C);
    int zMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2D);
    myImu.linear_acceleration.z=(float)((int16_t)(zLSB | zMSB<<8)>>4)/1000*9.81;

    myImu.header.stamp = ros::Time::now();
}


int PigpioSetup()
{
  char *addrStr = NULL;
  char *portStr = NULL;
  pi = pigpio_start(addrStr, portStr);
  return pi;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_publisher");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu/data_raw", 0);

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
  accel_setup();
  gyro_setup();
  //set our range message fixed data
  myImu.header.frame_id = "imu";

  //set accel covariance to generic ballpark guess.
    for(int i = 0; i<9; i++)
    {
        if(i==0 || i==4 || i==8)
        {
            myImu.linear_acceleration_covariance[i]=.0001;
            myImu.angular_velocity_covariance[i] = .0001;
        }
        else
        {
            myImu.linear_acceleration_covariance[i]=0;
            myImu.angular_velocity_covariance[i] = 0;
        }
        myImu.orientation_covariance[i] = 0;
    }
  //set absolute orientaion and angular velocity to "Do not use"
  myImu.orientation_covariance[0] = -1;

  ros::Rate loop_rate(10);
  while(ros::ok)
  {
    get_accel();
    get_gyro();
    pub.publish(myImu);
    loop_rate.sleep();
  }

  i2c_write_byte_data(pi, ACCEL_HANDLE, 0x20, 0x00);//puts accelerometer into sleep mode
  i2c_close(pi, ACCEL_HANDLE);
  i2c_write_byte_data(pi, GYRO_HANDLE, 0x20, 0);//puts gyro to sleep mode
  i2c_close(pi, GYRO_HANDLE);

  pigpio_stop(pi);
  return 0;
}

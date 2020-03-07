/*
*hello_roomba.cpp is a simple program with a few sample functions for communicating with your Roomba
*using the PIGPIO library. Note that you’ll have to study the PIGPIO documentation and the Roomba OI
*documentation for this to really make sense, these are here to make it easier for you to understand
*how the OI and PIGPIO work together.
*
*From the book Practical Robotics in C++ by Lloyd Brombach 2020
*/

#include "pigpiod_if2.h"

using namespace std;

int pi = -1;
int serHandle = -1;

//used to make sure roomba awake and listening
void wake(){
    int R1 = 23;  //This is assuming the wake relay is on pin 23
    set_mode(pi, R1, PI_OUTPUT);

    gpio_write(pi, R1, 0);     //pulse wake relay
    time_sleep(.1);
    gpio_write(pi, R1, 1);

    serial_write_byte(pi,serHandle,128);          //send start command
    time_sleep(.15);
    serial_write_byte(pi,serHandle,131);          //set to safe mode
    time_sleep(.15);
}

//drive in reverse at 80mm/sec
void rev()
{
   char driveString[] = {137, 255, 136, 0, 0};
   serial_write(pi, serHandle, driveString, 5);
}

//drive forward at 120mm/sec
void fwd()
{
   char driveString[] = {137, 0, 120, 127, 255};
   serial_write(pi, serHandle, driveString, 5);
}

//stops wheel motors
void stop()
{
   char driveString[] = {137, 0, 0, 0, 0};
   serial_write(pi, serHandle, driveString, 5);
}

//puts Roomba to sleep and frees program resources
void shutdown()
{
   serial_write_byte(pi,serHandle,133);
   time_sleep(.1);
   serial_close(pi, serHandle);
   pigpio_stop(pi);
}

//main() wakes Roomba, drives it forward for 5 seconds. Pauses
//for one second, reverses for 5 seconds, the shuts everything
//down
int main()
{
   char *addrStr = NULL;
   char *portStr = NULL;
   //handshake with daemon and get pi handle
   pi = pigpio_start(addrStr, portStr);

   //open serial port
   serHandle = serial_open(pi, "/dev/ttyUSB0",115200,0);

   wake();
   fwd();
   time_sleep(5);
   stop();
   time_sleep(1);
   rev();
   time_sleep(5);
   stop();
   shutdown();
}

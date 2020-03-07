/*
*roomba_sleep.cpp is a simple program to help "unfreeze" a roomba during eperimentation with the
*open interface. Sometimes test code will leave Roomba waiting for new data forever, so blasting
*it with the "sleep" command should satisfy the data requirement and eventuall put it to sleep mode.
*
*From the book Practical Robotics in C++ by Lloyd Brombach, 2020
*/


#include "pigpiod_if2.h"

using namespace std;


//puts Roomba to sleep
{
   int pi = -1;
   int serHandle = -1;
   char *addrStr = NULL;
   char *portStr = NULL;
   //handshake with daemon and get pi handle
   pi = pigpio_start(addrStr, portStr);

   //open serial port
   //serHandle = serial_open(pi, "/dev/ttyUSB0",115200,0);
   serHandle = serial_open(pi, "/dev/ttyAMA0",115200,0);

   for(int i = 0; i < 100; i++)
   {
        serial_write_byte(pi, serHandle, 133);
        time_sleep(.01);
   }

   time_sleep(.1);
   serial_close(pi, serHandle);
   pigpio_stop(pi);
}

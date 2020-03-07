/*
hello_button is a program to learn to read the input of a
Raspberry Pi GPIO pin with the PIGPIO library. This program
is an accompaniment to the book Practical Robotics in C++
written by Lloyd Brombach and published by Packt Publishing
*/

#include <pigpiod_if2.h>
#include <iostream>

const int LED = 5;
const int BUTTON = 27;

using namespace std;


int PigpioSetup()
{
   char *addrStr = NULL;
   char *portStr = NULL;
   //handshake with daemon and get pi handle
   int pi = pigpio_start(addrStr, portStr);

   //set the pin 6 mode and intialize to low
   set_mode(pi, LED, PI_OUTPUT);
   gpio_write(pi, LED, 0);

   //set pin 27 mode as input
   set_mode(pi, BUTTON, PI_INPUT);
   set_pull_up_down(pi, BUTTON, PI_PUD_UP);

   return pi;
}

int main()
{
  //initialize pipiod interface
  int pi = PigpioSetup();
  //check that handshake went ok
  if(pi>=0)
  {
    cout<<"daemon interface started ok at "<<pi<<endl;
  }
  else
  {
    cout<<"Failed to connect to PIGPIO Daemon - Try running sudo pigpiod and try again."<<endl;
    return -1;
  }

  while(1)
  {
      if(gpio_read(pi, BUTTON) == false)
      {
         //set pin 6 high to turn Led on
         gpio_write(pi, LED, 1);
         //sleep for 1.5 seconds
         time_sleep(1.5);
         //turn led off
         gpio_write(pi, LED, 0);
      }
  }

  //disconnect from pigpio daemon
  pigpio_stop(pi);
  return 0;
}

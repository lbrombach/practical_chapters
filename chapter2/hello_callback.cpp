/*
hello_callback is a generic program to learn to use GPIO pin event
callback functions with the PIGPIO library on a Raspberry pi.
This program is an accompaniment to the book Practical Robotics in C++
written by Lloyd Brombach and published by Packt Publishing.
*/

#include <iostream>
#include <pigpiod_if2.h>

using namespace std;

const int BUTTON = 27;


//this is the callback function that runs when a change of state happens on the monitored gpio pin
void button_event(int pi, unsigned int gpio, unsigned int edge, unsigned int foo)
{
    static int i = 0;
    cout<<"Button pressed. Press count = "<<i++<<endl;
}


int main()
{
   char *addrStr = NULL;
   char *portStr = NULL;
   int pi = pigpio_start(addrStr, portStr);
   if(pi>=0)
   {
      cout<<"daemon interface started ok at "<<pi<<endl;
   }
   else
   {
      cout<<"Failed to connect to PIGPIO Daemon - Try running sudo pigpiod and try again."<<endl;
      return -1;
   }

   //set pin 27 mode as input
   set_mode(pi, BUTTON, PI_INPUT);
   set_pull_up_down(pi, BUTTON, PI_PUD_UP);

   //initalizes a callback
   int callbackID=callback(pi, BUTTON, RISING_EDGE, button_event);

   //lets the program run for 60 seconds
   time_sleep(60);
   cout<<"60 seconds has elapsed. Program ending."<<endl;

   //cancel the callback and end the pigpio daemon interface
   callback_cancel(callbackID);
   pigpio_stop(pi);
   return 0;
}

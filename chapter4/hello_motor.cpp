/*
hello_motor is a program to learn to use a L298N dual
H Bridge motor driver with the PIGPIO library for the Raspberry Pi.
This program is an accompaniment to the book Practical Robotics in C++
written by Lloyd Brombach and published by Packt Publishing
*/


#include <iostream>
#include <pigpiod_if2.h>


//define our GPIO pin assignments
const int PWM_A = 21;
const int MOTOR_A_FWD = 26;
const int MOTOR_A_REV = 13;

//handshakes with Pigpio Daemon and sets up our pins.
int pigpio_setup()
{
    char *addrStr = NULL;
    char *portStr = NULL;
    const int pi = pigpio_start(addrStr, portStr);

    //next four lines sets up our pins. Remember that high is "off"
    //and we must drive in1 or in2 low to start the output to motor
    set_mode(pi,PWM_A, PI_OUTPUT);
    set_mode(pi,MOTOR_A_FWD, PI_OUTPUT);
    set_mode(pi,MOTOR_A_REV, PI_OUTPUT);

    //initializes motor off
    gpio_write(pi, MOTOR_A_FWD, 1);
    gpio_write(pi, MOTOR_A_REV, 1);
    return pi;
}

using namespace std;

int main()
{
    int pi = pigpio_setup();
    if(pi < 0)
    {
        cout<<"Failed to connect to Pigpio Daemon. Is it running?"<<endl;
        return -1;
    }


    //when you're ready to start the motor
    gpio_write(pi, MOTOR_A_FWD, 0);

    // starts a PWM signal to motor A enable at half speed
    set_PWM_dutycycle(pi, PWM_A, 127);
    time_sleep(3); //3 second delay
    //starts motor at full speed
    set_PWM_dutycycle(pi, PWM_A, 255);
    time_sleep(3);

    //stops the motor
    gpio_write(pi, MOTOR_A_FWD, 1);
    time_sleep(1);
    //repeats in reverse
    gpio_write(pi, MOTOR_A_REV, 0);
    set_PWM_dutycycle(pi, PWM_A, 127);
    time_sleep(3);
    set_PWM_dutycycle(pi, PWM_A, 255);
    time_sleep(3);
    gpio_write(pi, MOTOR_A_REV, 1);

    pigpio_stop(pi);
    return 0;
}

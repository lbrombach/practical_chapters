/*
hello_serial.cpp is a program to learn to use and test
serial communication with the PIGPIO library for the Raspberry Pi.
This program is an accompaniment to the book Practical Robotics in C++
written by Lloyd Brombach and published by Packt Publishing
*/

#include <iostream>
#include <pigpiod_if2.h>

using namespace std;

int main()
{
	//initialize connection with pigpio daemon
	char *addrStr = NULL;
	char *portStr = NULL;
	int pi=pigpio_start(addrStr, portStr);

	//open serial port connection
	int UARTHandle = serial_open(pi, "/dev/ttyAMA0",115200,0); //for use with GPIO serial pings 14 and 15
	
	//int UARTHandle = serial_open(pi, "/dev/ttyUSB0",115200,0); //for use with FTDI USB device
	cout<<"UARTHandle = " << UARTHandle<< endl;
	time_sleep(.1);

	//check serial buffer
	cout << "Data available start: " << serial_data_available(pi, UARTHandle)<< " bytes" << endl;

	//write a few test bytes
    serial_write_byte(pi,UARTHandle,6);
	serial_write_byte(pi,UARTHandle,'f');
	serial_write_byte(pi,UARTHandle,'F');

	//give time to transmit 
	time_sleep(.1);

	//check serial buffer again
	cout << "Data available after writing: " << serial_data_available(pi, UARTHandle)<< " bytes" << endl;

	//read and display one byte
	cout <<"Byte read = " << serial_read_byte(pi, UARTHandle)<< endl;

	//check serial buffer again
	cout << "Data available after reading a byte: " << serial_data_available(pi, UARTHandle)<< " bytes" << endl;

	//read and display last two bytes
    char inA = serial_read_byte(pi, UARTHandle);
	cout <<"Byte read = " << inA << endl;
	char inB = serial_read_byte(pi, UARTHandle);
    cout <<"Byte read = " << inB<< endl;

	//close serial device and terminate connection with pigpio daemon
	serial_close(pi, UARTHandle);
	pigpio_stop(pi);
	return 0;
}
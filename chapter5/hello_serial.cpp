#include <iostream>
#include <pigpiod_if2.h>

using namespace std;

int main()
{
	char *addrStr = NULL;
	char *portStr = NULL;
	int pi=pigpio_start(addrStr, portStr);

	int UARTHandle = serial_open(pi, "/dev/ttyAMA0",115200,0);
	//int UARTHandle = serial_open(pi, "/dev/ttyUSB0",115200,0);
	cout<<"UARTHandle = " << UARTHandle<< endl;
	time_sleep(.1);
	cout << "Data available start: " << serial_data_available(pi, UARTHandle)<< " bytes" << endl;
    	serial_write_byte(pi,UARTHandle,6);
    	serial_write_byte(pi,UARTHandle,'f');
    	serial_write_byte(pi,UARTHandle,'F');
	time_sleep(.1);

	cout << "Data available after writing: " << serial_data_available(pi, UARTHandle)<< " bytes" << endl;
	cout <<"Byte read = " << serial_read_byte(pi, UARTHandle)<< endl;

	cout << "Data available after reading a byte: " << serial_data_available(pi, UARTHandle)<< " bytes" << endl;
    	char inA = serial_read_byte(pi, UARTHandle);
    	cout <<"Byte read = " << inA << endl;
    	char inB = serial_read_byte(pi, UARTHandle);
    	cout <<"Byte read = " << inB<< endl;

	serial_close(pi, UARTHandle);
	pigpio_stop(pi);
	return 0;
}
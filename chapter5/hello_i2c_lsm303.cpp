#include <iostream>
#include <pigpiod_if2.h>

using namespace std;

const int LSM303_accel=0x19; //accelerometer address
const int I2Cbus=1; //RPi typical I2C bus. Find yours with "ls /dev/*i2c* "



int main()
{
char *addrStr = NULL;
	char *portStr = NULL;
	int pi=pigpio_start(addrStr, portStr);
	if(pi>=0)
    	{
    		cout<<"daemon interface started ok at "<<pi<<endl;
    	}
 	else
 	{
 		cout<<"no bueno, did you forget to start pigpio daemon (sudo pigpiod from terminal) ? "<<endl;
 	return -1;
 	}

	const int ACCEL_HANDLE=i2c_open(pi,I2Cbus, LSM303_accel,0);
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


    	int xLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x28);
    	int xMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x29);
    	float accelX=(float)((int16_t)(xLSB | xMSB<<8)>>4);

	int yLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2A);
    	int yMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2B);
    	float accelY=(float)((int16_t)(yLSB | yMSB<<8)>>4);

    	int zLSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2C);
    	int zMSB = (int)i2c_read_byte_data(pi, ACCEL_HANDLE, 0x2D);
    	float accelZ=(float)((int16_t)(zLSB | zMSB<<8)>>4);


	cout<<endl<<accelX<<"   "<<accelY<<"   "<<accelZ<<endl;


	if(accelX < 50 && accelX > -50 && accelY < 50 && accelY > -50)
    	{
     		cout<<"The device is fairly level"<<endl;
    	}
	else
    	{
     		cout<<"The device is not level"<<endl;
    	}

	i2c_write_byte_data(pi, ACCEL_HANDLE, 0x20, 0x00);//puts accelerometer into sleep mode
	i2c_close(pi, ACCEL_HANDLE);

	pigpio_stop(pi);

    	return 0;
}

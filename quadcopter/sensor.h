#ifndef _INCLUDE_SENSOR_
#define _INCLUDE_SENSOR_

#include <stdio.h>
#include "BASIC.h"

#define ACCRES	0.001f
#define GYRORES	0.0001526f

typedef struct Sensor {
	float ax;
	float ay;
	float az;
	float wx;
	float wy;
	float wz;
	int mx;
	int my;
	int mz;
	float alt;
} Sensor;

void initializeSensor(void) {
	pioInit();
	
	// pin assignments for ultrasound
	pinMode(21, OUTPUT);
	pinMode(20, INPUT);
	
	// initialize i2c pins and clk frequency
	i2cInit(200000);
	
	// power on accelerometer
	i2cClearBits();
	i2cSlaveAdr(0b0011001);
	i2cPowerOn(0b00100000, 0b00110111);

	// power on gyroscope
	i2cClearBits();
	i2cSlaveAdr(0b1101011);
	i2cPowerOn(0b00100000, 0b00001111);

	// power on magnetometer
	i2cClearBits();
	i2cSlaveAdr(0b0011110);
	i2cPowerOn(0b00000010, 0b00000000);
}

// ULTRASOUND DATA METHODS

void getHeight(Sensor *sensor) {
	digitalWrite(21, 1);
	delayMicrosecs(10);
	digitalWrite(21, 0);

	while(!digitalRead(20)){};

	int time1 = getTime();
	while(digitalRead(20)){};
	int time2 = getTime();

	float range = (float) (time2 - time1) / 1000000 * 340 / 2; // in meters
	printf("range = %f\n", range);

	sensor->alt = range;
}

// IMU DATA METHODS

int convert12to32(char mb, char lb) {
	int x = (mb >> 7 == 0) ? ((mb << 4) | (lb >> 4)) : (0xFFFFF000 | ((mb << 4) | (lb >> 4)));
	return x;
}

int convert16to32(char mb, char lb) {
	int x = (mb >> 7 == 0) ? ((mb << 8) | lb) : (0xFFFF0000 | ((mb << 8) | lb));
	return x;
}

void getIMU(Sensor *sensor) {
	char accData[6] = {1,2,3,4,5,6};
	char gyroData[6] = {7,8,9,10,11,12};
	char magData[6] = {13,14,15,16,17};

	i2cClearBits();
	i2cSlaveAdr(0b0011001);
	while(!i2cRead(0b10101000, accData, 6)){};

	i2cClearBits();
	i2cSlaveAdr(0b1101011);
	while(!i2cRead(0b10101000, gyroData, 6)){};

	i2cClearBits();
	i2cSlaveAdr(0b0011110);
	while(!i2cRead(0b00000011, magData, 6)){};

	int rawax = convert12to32(accData[1], accData[0]); 
	int raway = convert12to32(accData[3], accData[2]); 
	int rawaz = convert12to32(accData[5], accData[4]); 
	int rawwx = convert16to32(gyroData[1], gyroData[0]);
	int rawwy = convert16to32(gyroData[3], gyroData[2]);
	int rawwz = convert16to32(gyroData[5], gyroData[4]);
	int rawmx = convert16to32(magData[0], magData[1]);
	int rawmy = convert16to32(magData[4], magData[5]);
	int rawmz = convert16to32(magData[2], magData[3]);
	sensor->ax = rawax * ACCRES;
	sensor->ay = raway * ACCRES;
	sensor->az = rawaz * ACCRES;
	sensor->wx = rawwx * GYRORES;
	sensor->wy = rawwy * GYRORES;
	sensor->wz = rawwz * GYRORES;
	sensor->mx = rawmx;
	sensor->my = rawmy;
	sensor->mz = rawmz;
	
	// Remove print statements later
	/*printf("accData = %d, %d, %d\n", ((accData[1] << 2) | accData[0] >> 6),
		((accData[3] << 2) | accData[2] >> 6),
		((accData[5] << 2) | accData[4] >> 6));
	printf("gyroData = %d, %d, %d\n", ((gyroData[1] << 2) | gyroData[0] >> 6),
		((gyroData[3] << 2) | gyroData[2] >> 6),
		((gyroData[5] << 2) | gyroData[4] >> 6));
	printf("accData = %d, %d, %d, %d, %d, %d\n", accData[0], accData[1],
		accData[2], accData[3], accData[4], accData[5]);
	printf("gyroData = %d, %d, %d, %d, %d, %d\n", gyroData[0], gyroData[1],
		gyroData[2], gyroData[3], gyroData[4], gyroData[5]);*/
}


Sensor getData(void) {
	Sensor sensor;
	
	getIMU(&sensor);
	getHeight(&sensor);
	
	return sensor;
}
#endif
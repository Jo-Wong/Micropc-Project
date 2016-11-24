#include <stdio.h>
#include <stdlib.h>
#include "sensor.h"

void main(void) {
	initializeSensor();
	Sensor mySensor;
	mySensor = getData();

	//printf("altitude = %f", mySensor.alt);
	printf("wx = %f\n", mySensor.wx);
	printf("wy = %f\n", mySensor.wy);
	printf("wz = %f\n", mySensor.wz);
	printf("ax = %f\n", mySensor.ax);
	printf("ay = %f\n", mySensor.ay);
	printf("az = %f\n", mySensor.az);
}

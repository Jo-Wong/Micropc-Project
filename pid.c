#include <stdio.h>
#include <math.h>
#include "sensor.h"

typedef struct {
	float currValue;
	float prevError;
} State;

typedef struct {
	float globalWX;
	float globalWY;
	float globalWZ;
} Global;

typedef struct {
	float thrust; // considering only z-axis
	float tauX;
	float tauY;
	float tauZ;
} Dynamics;

typedef struct {
	float w1;
	float w2;
	float w3;
	float w4;
} Output;

#define Z_TARGET	5.0f // Move quad to height of 5 meters
#define GX_TARGET	0.0f
#define GY_TARGET	0.0f
#define GZ_TARGET	0.0f

#define Z_PROP_GAIN		0.0f
#define GX_PROP_GAIN	0.0f
#define GY_PROP_GAIN	0.0f
#define GZ_PROP_GAIN	0.0f

#define Z_DERV_GAIN		0.0f
#define GX_DERV_GAIN	0.0f
#define GY_DERV_GAIN	0.0f
#define GZ_DERV_GAIN	0.0f

#define IXX	0.005075f
#define IYY	0.005048f
#define IZZ	0.0260908f

#define k	0.000511f;
#define b	0.000005f;
#define l	0.2286f;

#define CONSTANT1	1/(4*k) 	//1/4k
#define CONSTANT2	1/(2*k*l) 	//1/2kl
#define CONSTANT3	1/(4*b) 	//1/4b

#define GRAVITY		9.80665f
#define UPDATE_PERIOD	0.1f 	//100ms
#define MASS		1.1141f		// in kg
#define MAX_RPM		35000 		// 12 pole motor

void main(void) {
	// Initialize our states
	State zState = {.currValue = 0, .prevError = 0};
	State gxState = {.currValue = 0, .prevError = 0};
	State gyState = {.currValue = 0, .prevError = 0};
	State gzState = {.currValue = 0, .prevError = 0};

	// Declare Sensor structure
	Sensor sensor;
	
	// Declare Global structure
	Global globalW;
	
	// Declare Dynamics strucutre
	Dynamics dynamics;
	
	// Declare Output structure
	Output output;
	
	// Calculate the current error of our states
	float zError = zState.currValue - Z_TARGET;
	float gxError = gxState.currValue - GX_TARGET;
	float gyError = gyState.currValue - GY_TARGET;
	float gzError = gxState.currValue - GZ_TARGET;
	
	// Calculate the needed thrust and torques
	dynamics.thrust = MASS / cos(gxState.currValue) / cos(gyState.currValue) * (GRAVITY + Z_DERV_GAIN * (zError - zState.prevError) + Z_PROP_GAIN * zError); 
	dynamics.tauX = IXX * (GX_DERV_GAIN * (gxError - gxState.prevError) + GX_PROP_GAIN * gxError);
	dynamics.tauY = IYY * (GY_DERV_GAIN * (gyError - gyState.prevError) + GY_PROP_GAIN * gyError);
	dynamics.tauZ = IZZ * (GZ_DERV_GAIN * (gzError - gzState.prevError) + GZ_PROP_GAIN * gzError);

	// Store the current error of our states
	zState.prevError = zError;
	gxState.prevError = gxError;
	gyState.prevError = gyError;
	gzState.prevError = gzError;
	
	// Calculate the needed thrust and torques
	dynamics.thrust = MASS / cos(gxState.currValue) / cos(gyState.currValue) * (GRAVITY + Z_DERV_GAIN * (zError - zState.prevError) / UPDATE_PERIOD + Z_PROP_GAIN * zError); 
	dynamics.tauX = IXX * (GX_DERV_GAIN * (gxError - gxState.prevError) / UPDATE_PERIOD + GX_PROP_GAIN * gxError);
	dynamics.tauY = IYY * (GY_DERV_GAIN * (gyError - gyState.prevError) / UPDATE_PERIOD + GY_PROP_GAIN * gyError);
	dynamics.tauZ = IZZ * (GZ_DERV_GAIN * (gzError - gzState.prevError) / UPDATE_PERIOD + GZ_PROP_GAIN * gzError);
		
	char value1 = output.w1 / MAX_RPM * 255;
	char value2 = output.w2 / MAX_RPM * 255;
	char value3 = output.w3 / MAX_RPM * 255;
	char value4 = output.w4 / MAX_RPM * 255;
	
	// wait for some time
	while (UPDATE_PERIOD) {
		//remember: 	delayMicrosecs(60*1000); // in ultimate
		
		// Get sensor readings
		sensor = getData();
		
		// Determine inertial angular velocities
		globalW.globalWX = sensor.wx * cos(gyState.currValue) - sensor.wz * sin(gyState.currValue);
		globalW.globalWY = sensor.wy + sensor.wx * sin(gyState.currValue) * tan(gxState.currValue) + sensor.wz * cos(gyState.currValue) * tan(gxState.currValue);
		globalW.globalWZ = sensor.wx * sin(gyState.currValue)/cos(gxState.currValue) + sensor.wz * cos(gyState.currValue) / cos(gxState.currValue);

		// Store the current value of our states
		zState.currValue = sensor.alt;
		gxState.currValue = globalW.globalWX * UPDATE_PERIOD;
		gyState.currValue = globalW.globalWY * UPDATE_PERIOD;
		gzState.currValue = globalW.globalWZ * UPDATE_PERIOD;
		
		// Calculate the current error of our states
		float zError = zState.currValue - Z_TARGET;
		float gxError = gxState.currValue - GX_TARGET;
		float gyError = gyState.currValue - GY_TARGET;
		float gzError = gxState.currValue - GZ_TARGET;
		
		// Calculate the needed thrust and torques
		dynamics.thrust = MASS / cos(gxState.currValue) / cos(gyState.currValue) * (GRAVITY + Z_DERV_GAIN * (zError - zState.prevError) / UPDATE_PERIOD + Z_PROP_GAIN * zError); 
		dynamics.tauX = IXX * (GX_DERV_GAIN * (gxError - gxState.prevError) / UPDATE_PERIOD + GX_PROP_GAIN * gxError);
		dynamics.tauY = IYY * (GY_DERV_GAIN * (gyError - gyState.prevError) / UPDATE_PERIOD + GY_PROP_GAIN * gyError);
		dynamics.tauZ = IZZ * (GZ_DERV_GAIN * (gzError - gzState.prevError) / UPDATE_PERIOD + GZ_PROP_GAIN * gzError);

		// Store the current error of our states
		zState.prevError = zError;
		gxState.prevError = gxError;
		gyState.prevError = gyError;
		gzState.prevError = gzError;
		
		// Calculate the angular velocities for our motors
		output.w1 = dynamics.thrust * CONSTANT1 - dynamics.tauX * CONSTANT2 - dynamics.tauZ * CONSTANT3;
		output.w2 = dynamics.thrust * CONSTANT1 - dynamics.tauY * CONSTANT2 + dynamics.tauZ * CONSTANT3;
		output.w3 = dynamics.thrust * CONSTANT1 + dynamics.tauX * CONSTANT2 - dynamics.tauZ * CONSTANT3;
		output.w4 = dynamics.thrust * CONSTANT1 + dynamics.tauY * CONSTANT2 + dynamics.tauZ * CONSTANT3;
	
		char value1 = output.w1 / MAX_RPM * 255;
		char value2 = output.w2 / MAX_RPM * 255;
		char value3 = output.w3 / MAX_RPM * 255;
		char value4 = output.w4 / MAX_RPM * 255;
	}
}

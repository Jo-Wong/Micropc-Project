/* Written for MicroPs final project 
 * pid.c  
*/
#include <sys/select.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// to use M_PI
// #define _USE_MATH_DEFINES
#include <math.h>

#include "sensor.h"
#include "complementary.h"
#include "esc_output.h"
#include "BASIC.h"

typedef struct Errors {
	float angX;
	float angY;
	float angZ;
} Errors;

typedef struct Angles {
	float angX; // pitch
	float angY; // roll
	float angZ; // yaw
} Angles;

typedef struct Output{
	float w1;
	float w2;
	float w3;
	float w4;
} Output;

#define GX_TARGET	0.0f
#define GY_TARGET	0.0f
#define GZ_TARGET	0.0f

#define KP	3.0f
#define KI	5.5f
#define KD	4.0f

#define IXX	0.005f
#define IYY	0.005f
#define IZZ	0.02f

#define B   0.0000001f
#define K   0.000003f
#define L   0.15f  //quadcopter radius in m

#define MOTOR_KV    1110.0f

#define GRAVITY		        9.80665f
#define UPDATE_PERIOD	    0.100f
#define UPDATE_PERIOD_US	100000
#define MASS		        1.18f
#define M_PI                3.14159265359

void pid_controller(Sensor sensor, Output *outs, Angles *thetadot, Angles *theta, Angles *thetaInt, Errors e);
void calc_outputs(Output *out, Errors e, float thrust);
void send_motor_values(Output motors, char* motor_numbers);

void main(void) {

	// For handling user input
	fd_set readfds;
    FD_ZERO(&readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    char user_input[50];
	
	// Declare Sensor struct
	Sensor sensor;
	// Initialize everything to zero
	Errors e = {0, 0, 0};
	// Declare angle structs to hold wx, wy, wz for theta(local) and omega(global)
	Angles theta, thetadot, thetaInt = {0, 0, 0};
				// Angles omega, omegadot = {.angX = 0, .angY = 0, .angZ = 0};
	// Declare Output struct
	Output out = {0, 0, 0, 0};
	// Numbers for motor output
	char  motorNumbers[4] = {0, 0, 0, 0};
	// For checking delay in PID loop
	int micros = 0; 
	
	// setup ESCs
	setup_ESCs();
	// setup sensor
	initializeSensor();
	
	while(1)
	{
		micros = getTime();
		
		sensor = getData();
		
		thetadot.angX = sensor.wx;
		thetadot.angY = sensor.wy;
		thetadot.angZ = sensor.wz;
		
		pid_controller(sensor, &out, &thetadot, &theta, &thetaInt, e);
		send_motor_values(out, motorNumbers);
		
		// Check for user input only when there is extra time
		while((getTime() - micros) < UPDATE_PERIOD_US)
		{
			FD_SET(STDIN_FILENO, &readfds);
			if (select(1, &readfds, NULL, NULL, &timeout))
			{
				scanf("%s", user_input); 
			// printf("Message: %s\n", message);
			}
			
			if(user_input[0] == 'u')
			{
				// do something...
			}
			// printf("...\n");
			// delayMicrosecs(10);	
		}
	}
	
}
	
void pid_controller(Sensor sensor, Output *outs, Angles *thetadot, Angles *theta, Angles *thetaInt, Errors e)
{
// Calculate total thrust
	float thrust = MASS * GRAVITY/ (K * cos(theta->angX * theta->angY));
	
// Prevent large thetaInt change / windup
	if(abs(thetaInt->angX) > 0.01 || abs(thetaInt->angY) > 0.01 || abs(thetaInt->angZ) > 0.01)
	{
		thetaInt->angX = 0;
		thetaInt->angY = 0;
		thetaInt->angZ = 0;
	}
	
// Calculate all errors
	e.angX = (KD * thetadot->angX) + (KP * theta->angX) - (KI * thetaInt->angX);
	e.angY = (KD * thetadot->angY) + (KP * theta->angY) - (KI * thetaInt->angY);
	e.angZ = (KD * thetadot->angZ) + (KP * theta->angZ) - (KI * thetaInt->angZ);
	
// Calculate all outputs (motor rpm -> esc voltage)
	calc_outputs(outs, e, thrust);
	
// Save past pitch and roll for use in filter later
	float preThetaX = theta->angX;
	float preThetaY = theta->angY;
	
// Update the integrals (angle, angleIntegrated)
	theta->angX = theta->angX + (UPDATE_PERIOD*thetadot->angX);
	theta->angY = theta->angY + (UPDATE_PERIOD*thetadot->angY);
	// theta.angZ = theta.angZ + (UPDATE_PERIOD*thetadot.angZ);		
	complementary_filter(&(theta->angX), &(theta->angY), preThetaX, preThetaY, sensor.ax, sensor.ay, sensor.az, UPDATE_PERIOD);
	theta->angZ = tilt_compensation(theta->angX, theta->angY, sensor.mx, sensor.my, sensor.mz);
	
	thetaInt->angX = thetaInt->angX + (UPDATE_PERIOD*theta->angX);
	thetaInt->angY = thetaInt->angY + (UPDATE_PERIOD*theta->angY);
	thetaInt->angZ = thetaInt->angZ + (UPDATE_PERIOD*theta->angZ);	
	
}
	
void calc_outputs(Output *out, Errors e, float thrust)
{
	out->w1 = thrust/4 - (2*B*e.angX*IXX + e.angZ*IZZ*K*L)/(4*B*K*L);
	out->w2 = thrust/4 - e.angZ*IZZ/(4*B) - (e.angY*IYY)/(2*K*L);
	out->w3 = thrust/4 - (-2*B*e.angX*IXX + e.angZ*IZZ*K*L)/(4*B*K*L);
	out->w4 = thrust/4 + e.angZ*IZZ/(4*B) + (e.angY*IYY)/(2*K*L);

	// These are angular velocities in rad/s, convert to voltage using kV (rpm/V)
	// Convert to throttle level (V/total V) a percentage
	out->w1 = out->w1 /(2.0*M_PI*60.0f) / MOTOR_KV / 11.1f;
	out->w2 = out->w2 /(2.0*M_PI*60.0f) / MOTOR_KV / 11.1f;
	out->w3 = out->w3 /(2.0*M_PI*60.0f) / MOTOR_KV / 11.1f;
	out->w4 = out->w4 /(2.0*M_PI*60.0f) / MOTOR_KV / 11.1f;
}
	
void send_motor_values(Output motors, char* motor_numbers)
{
	motor_numbers[0] = (char)(motors.w1*255); // cast floats to ints
	motor_numbers[1] = (char)(motors.w2*255);
	motor_numbers[2] = (char)(motors.w3*255);
	motor_numbers[3] = (char)(motors.w4*255);
	
	writeOut(motor_numbers);
}
/*
float abs(float x)
{
	if(x < 0) { x = -x; }
	return x;
}
*/

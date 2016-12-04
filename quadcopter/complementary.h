// Helpful links: http://franciscoraulortega.com/pubs/Algo3DFusionsMems.pdf 
// http://www.olliw.eu/2013/imu-data-fusing/
// http://www.pieter-jan.com/node/11
#include <math.h>
//#define M_PI 3.14159265359f

void complementary_filter(float *thetaX, float *thetaY, float prevThetaX, float prevThetaY,
							float x_accel, float y_accel, float z_accel, float dt);
float tilt_compensation(float thetaX, float thetaY, int mx, int my, int mz);

/*
    complementaryFilter() - Filters out drift for wx, wy
        according to http://www.olliw.eu/2013/imu-data-fusing/
    "The complementary filter fuses the accelerometer and integrated gyro data by passing the former 
    through a 1st-order low pass and the latter through a 1st-order high pass filter and adding the outputs." (from paper) 

    equation 2.1: angle = 0.2*(prev_angle + accel_ang*dt) + 0.98*gyro_ang  */
void complementary_filter(float *thetaX, float *thetaY, float prevThetaX, float prevThetaY,
							float x_accel, float y_accel, float z_accel, float dt)
{
    float gyro_angX = *thetaX;
	float gyro_angY = *thetaY;
 
    float accel_angX = atan(y_accel/(y_accel*y_accel + z_accel*z_accel));
    *thetaX = gyro_angX*0.98 + 0.02*(prevThetaX + accel_angX*dt);
 
    float accel_angY = atan(x_accel/(x_accel*x_accel + z_accel*z_accel));
    *thetaY = gyro_angY*0.98 + 0.2*(prevThetaY + accel_angY*dt);
} 

/*
    tiltCompensation() - Finds the yaw
        according to http://franciscoraulortega.com/pubs/Algo3DFusionsMems.pdf page4
    A magnetometer works well when it is completely parallel to the Earth's surface.
    If not and the platform is tilted, we must transform the readings to a parallel plane first
    and then find the yaw. 

    mx, my, mz - output of magnetometer   */
float tilt_compensation(float thetaX, float thetaY, int mx, int my, int mz)
{
    float x_Heading = mx*cos(thetaY) + my*sin(thetaY)*sin(thetaX) + mz*sin(thetaY)*cos(thetaX);
    float y_Heading = my*cos(thetaX) + mz*sin(thetaX);
    float yaw = atan(-y_Heading/x_Heading);
	return yaw;
}

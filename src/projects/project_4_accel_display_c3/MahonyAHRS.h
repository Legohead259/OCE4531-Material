//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h
#include <math.h>
#include "utility/imumaths.h"

//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony {
private:
	imu::Quaternion Q; 
	float twoKp;		// 2 * proportional gain (Kp)
	float twoKi;		// 2 * integral gain (Ki)
	float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
	float invSampleFreq;
	imu::Vector<3> eulerAngles;
	float roll, pitch, yaw;
	char anglesComputed;
	static float invSqrt(float x);
	void computeAngles();

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony(float sampleFreq=512.0f, float Kp=1.0f, float Ki=0.3f);
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	float getRoll() {
		if (!anglesComputed) computeAngles();
		return roll * 57.29578f;
	}
	float getPitch() {
		if (!anglesComputed) computeAngles();
		return pitch * 57.29578f;
	}
	float getYaw() {
		if (!anglesComputed) computeAngles();
		return yaw * 57.29578f + 180.0f;
	}
	imu::Vector<3> getEulerAngles() {
		return imu::Vector<3>(roll, pitch, yaw);
	}
	float getRollRadians() {
		if (!anglesComputed) computeAngles();
		return roll;
	}
	float getPitchRadians() {
		if (!anglesComputed) computeAngles();
		return pitch;
	}
	float getYawRadians() {
		if (!anglesComputed) computeAngles();
		return yaw;
	}
	void getQuaternionComps(float arr[4]) {
		arr[0] = q0;
		arr[1] = q1;
		arr[2] = q2;
		arr[3] = q3;
	}
	imu::Quaternion getQuaternion() {
		return imu::Quaternion(q0, q1, q2, q3);
	}
};

#endif

/*
 * Run an example of Kalman filter.
 * This example simulates a sinusoidal position of an object.
 * The 'SIMULATOR_' functions below simulate the physical process and its measurement with sensors. 
 * Results are printed on Serial port. You can use 'kalman_full.py' to analyse them with Python.
 * 
 * Author:
 *  R.JL. Fétick
 *  
 * Revision:
 *  31 Aug 2019 - Creation
 * 
 */

#include <Kalman.h>
// MPU6050 Instantiation
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
#include <math.h>

Adafruit_MPU6050 mpu;
sensors_event_t a, g, t; // Acceleration event holder

using namespace BLA;

//------------------------------------
/****  MODELIZATION PARAMETERS  ****/
//------------------------------------

#define Nstate 9 // position (XYZ), speed (XYZ), acceleration (XYZ)
#define Nobs 3   // acceleration (XYZ)

// measurement std of the noise
//#define n_p 0.3 // position measurement noise
#define n_a 5.0 // acceleration measurement noise

// model std (1/inertia)
#define m_p 0.1
#define m_s 0.1
#define m_a 0.8

BLA::Matrix<Nobs> obs; // observation vector
KALMAN<Nstate,Nobs> K; // your Kalman filter
unsigned long T; // current time
float DT; // delay between two updates of the filter

// Note: I made 'obs' a global variable so memory is allocated before the loop.
//       This might provide slightly better speed efficiency in loop.

//------------------------------------
/****        SETUP & LOOP       ****/
//------------------------------------

void setup() {

  Serial.begin(115200);

  Serial.print("Initializing IMU...");
  if (!mpu.begin()) { // Try to initialize!
      Serial.println("Failed to find MPU6050 chip");
      while (true); // Block further code execution
  }
  Serial.println("done!"); Serial.println();

  // time evolution matrix (whatever... it will be updated inloop)
  //       P  | S  | A  | P  | S  | A  | P  | S  | A
  K.F = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // X
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, // Y
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, // Z
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  // measurement matrix (position, velocity, acceleration)
  K.H = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,    // X
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,    // Y
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };  // Z
  
  // measurement covariance matrix
  K.R = { n_a*n_a,  0.0,      0.0,
          0.0,      n_a*n_a,  0.0,
          0.0,      0.0,      n_a*n_a };
  
  // model covariance matrix
  K.Q = { m_p*m_p, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // X
          0.0, m_s*m_s, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, m_a*m_a, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, m_p*m_p, 0.0, 0.0, 0.0, 0.0, 0.0, // Y
          0.0, 0.0, 0.0, 0.0, m_s*m_s, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, m_a*m_a, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m_p*m_p, 0.0, 0.0, // Z
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m_s*m_s, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, m_a*m_a };
  
  T = millis();
}

void loop() {
  
  // TIME COMPUTATION
  DT = (millis()-T)/1000.0;
  T = millis();

  // UPDATE STATE EQUATION
  // Here we make use of the Taylor expansion on the (position,speed,acceleration)
  // position_{k+1}     = position_{k} + DT*speed_{k} + (DT*DT/2)*acceleration_{k}
  // speed_{k+1}        = speed_{k} + DT*acceleration_{k}
  // acceleration_{k+1} = acceleration_{k}
  K.F = { 1.0, DT,  DT*DT/2,  0.0, 0.0, 0.0,      0.0, 0.0, 0.0,
          0.0, 1.0, DT,       0.0, 0.0, 0.0,      0.0, 0.0, 0.0,
          0.0, 0.0, 1.0,      0.0, 0.0, 0.0,      0.0, 0.0, 0.0,
          0.0, 0.0, 0.0,      1.0, DT,  DT*DT/2,  0.0, 0.0, 0.0,
          0.0, 0.0, 0.0,      0.0, 1.0, DT,       0.0, 0.0, 0.0,
          0.0, 0.0, 0.0,      0.0, 0.0, 1.0,      0.0, 0.0, 0.0,
          0.0, 0.0, 0.0,      0.0, 0.0, 0.0,      1.0, DT,  DT*DT*2,
          0.0, 0.0, 0.0,      0.0, 0.0, 0.0,      0.0, 1.0, DT,
          0.0, 0.0, 0.0,      0.0, 0.0, 0.0,      0.0, 0.0, 1.0};

  // UPDATE THE MEASUREMENTS
  pollIMU();
  
  // APPLY KALMAN FILTER
  K.update(obs);

  // PRINT RESULTS: true state, measurements, estimated state, posterior covariance
  // The most important variable for you might be the estimated state 'K.x'
//  Serial << state << ' ' << obs << ' ' << K.x << ' ' << K.P << '\n';
  Serial << obs << ','; // [a_x, a_y, a_z]
  Serial << K.x << ','; // [x_p, x_s, x_a, y_p, y_s, y_a, z_p, z_s, z_a]
//  Serial << K.P << '\n';
  Serial << '\n';

  delay(1000/52.0); // Run at ~52 Hz
}

//------------------------------------
/****     SIMULATOR FUNCTIONS   ****/
//------------------------------------

void pollIMU(){
  mpu.getEvent(&a, &g, &t);
  BLA::Matrix<Nobs> data;
  data(0) = a.acceleration.x;
  data(1) = a.acceleration.y;
  data(2) = a.acceleration.z;
  obs = data; // measurement
}

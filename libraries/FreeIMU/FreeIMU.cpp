/*
FreeIMU.cpp - A libre and easy to use orientation sensing library for Arduino
Copyright (C) 2011 Fabio Varesano <fabio at varesano dot net>


This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <inttypes.h>
//#define DEBUG
#include "WProgram.h"
#include "FreeIMU.h"
// #include "WireUtils.h"
#include "DebugUtils.h"

//----------------------------------------------------------------------------------------------------
// Definitions

#define Kp 2.0f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f   // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.02f   // half the sample period


FreeIMU::FreeIMU() {
  acc = ADXL345();
  gyro = ITG3200();
  magn = HMC58X3();
  
  // initialize quaternion
  q0 = 1;
  q1 = 0;
  q2 = 0;
  q3 = 0;
  exInt = 0;
  eyInt = 0;
  ezInt = 0;
  lastUpdate = 0;
  now = 0;
}

void FreeIMU::init() {
  init(FIMU_ADXL345_DEF_ADDR, FIMU_ITG3200_DEF_ADDR, false, 20);
}

void FreeIMU::init(bool fastmode) {
  init(FIMU_ADXL345_DEF_ADDR, FIMU_ITG3200_DEF_ADDR, fastmode, 20);
}

void FreeIMU::init(bool fastmode, int newperiod) {
  // use default addresses
  init(FIMU_ADXL345_DEF_ADDR, FIMU_ITG3200_DEF_ADDR, fastmode, newperiod);
}


void FreeIMU::init(int acc_addr, int gyro_addr, bool fastmode, int newperiod) {
  delay(5);
  
  
  // disable internal pullups of the ATMEGA which Wire enable by default
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
  #else
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    cbi(PORTD, 0);
    cbi(PORTD, 1);
  #endif
 
  
  if(fastmode) { // switch to 400KHz I2C - eheheh
    TWBR = ((16000000L / 400000L) - 16) / 2; // see twi_init in Wire/utility/twi.c
    // TODO: make the above usable also for 8MHz arduinos..
  }
  
  period = newperiod;
  halfT = newperiod / (1000.0 * 2); // store the half of the period expressed in seconds
  
  // init ADXL345
  acc.init(acc_addr);
  // init ITG3200
  gyro.init(gyro_addr);
  // calibrate the ITG3200
  gyro.zeroCalibrate(64,5);
  
  // init HMC5843
  magn.init(false); // Don't set mode yet, we'll do that later on.
  // Calibrate HMC using self test, not recommended to change the gain after calibration.
  magn.calibrate(1); // Use gain 1=default, valid 0-7, 7 not recommended.
  // Single mode conversion was used in calibration, now set continuous mode
  magn.setMode(0);
  delay(10);
  magn.setDOR(B110);
}


void FreeIMU::getRawValues(int * raw_values) {
  acc.readAccel(&raw_values[0], &raw_values[1], &raw_values[2]);
  gyro.readGyroRaw(&raw_values[3], &raw_values[4], &raw_values[5]);
  magn.getValues(&raw_values[6], &raw_values[7], &raw_values[8]);
}


void FreeIMU::getValues(float * values) {  
  int accval[3];
  acc.readAccel(&accval[0], &accval[1], &accval[2]);
  values[0] = ((float) accval[0]);
  values[1] = ((float) accval[1]);
  values[2] = ((float) accval[2]);
  
  gyro.readGyro(&values[3]);
  
  magn.getValues(&values[6]);
}



//=====================================================================================================
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//
//=====================================================================================================
void FreeIMU::AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  // auxiliary variables to reduce number of repeated operations
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  // normalise the measurements
  
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;
  
  
  now = millis();
  halfT = (now - lastUpdate) / 2000.0;
  lastUpdate = now;
  
  /*
  norm = invSqrt(ax*ax + ay*ay + az*az);
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;
  */
  
  norm = sqrt(mx*mx + my*my + mz*mz);          
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;
  
  /*
  norm = invSqrt(mx*mx + my*my + mz*mz);
  mx = mx * norm;
  my = my * norm;
  mz = mz * norm;
  */
  
  // compute reference direction of flux
  hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;     
  
  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
  
  // integral error scaled integral gain
  exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;
  
  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;
  
  // integrate quaternion rate and normalise
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  // normalise quaternion
  
 
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
  
/*
  norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
  */
}

void FreeIMU::getQ(float * q) {
  float val[9];
  getValues(val);
  
  DEBUG_PRINT(val[3] * M_PI/180);
  DEBUG_PRINT(val[4] * M_PI/180);
  DEBUG_PRINT(val[5] * M_PI/180);
  DEBUG_PRINT(val[0]);
  DEBUG_PRINT(val[1]);
  DEBUG_PRINT(val[2]);
  DEBUG_PRINT(val[6]);
  DEBUG_PRINT(val[7]);
  DEBUG_PRINT(val[8]);
  
  // gyro values are expressed in deg/sec, the * M_PI/180 will convert it to radians/sec
  AHRSupdate(val[3] * M_PI/180, val[4] * M_PI/180, val[5] * M_PI/180, val[0], val[1], val[2], val[6], val[7], val[8]);
  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

// Returns the Euler angles in radians defined with the Aerospace sequence.
// See Sebastian O.H. Madwick report 
// "An efficient orientation filter for inertial and intertial/magnetic sensor arrays" Chapter 2 Quaternion representation
void FreeIMU::getEuler(float * angles) {
  float q[4]; // quaternion
  getQ(q);
  angles[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI; // psi
  angles[1] = -asin(2 * q[1] * q[3] + 2 * q[0] * q[2]) * 180/M_PI; // theta
  angles[2] = atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1) * 180/M_PI; // phi
}



void FreeIMU::getYawPitchRoll(float * ypr) {
  float q[4]; // quaternion
  float gx, gy, gz; // estimated gravity direction
  getQ(q);
  
  gx = 2 * (q[1]*q[3] - q[0]*q[2]);
  gy = 2 * (q[0]*q[1] + q[2]*q[3]);
  gz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
  
  ypr[0] = atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0]*q[0] + 2 * q[1] * q[1] - 1) * 180/M_PI;
  ypr[1] = atan(gx / sqrt(gy*gy + gz*gz))  * 180/M_PI;
  ypr[2] = atan(gy / sqrt(gx*gx + gz*gz))  * 180/M_PI;
}


float invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}

/*
FreeIMU.h - A libre and easy to use orientation sensing library for Arduino
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

#include <Wire.h>
#include "WProgram.h"
#include <ADXL345.h>
#include <HMC58X3.h>
#include <ITG3200.h>


#ifndef FreeIMU_h
#define FreeIMU_h

// default I2C 7-bit addresses of the sensors
#define FIMU_ADXL345_DEF_ADDR ADXL345_ADDR_ALT_LOW // SDO connected to GND
//#define FIMU_ADXL345_DEF_ADDR ADXL345_ADDR_ALT_HIGH // SDO connected to GND
#define FIMU_ITG3200_DEF_ADDR ITG3200_ADDR_AD0_LOW // AD0 connected to GND
// HMC5843 address is fixed so don't bother to define it

// ITG3200 constants
#define FIMU_ITG3200_SMPLRT_DIV 0x15
#define FIMU_ITG3200_DLPF_FS 0x16
#define FIMU_ITG3200_INT_CFG 0x17
#define FIMU_ITG3200_PWR_MGM 0x3E

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

class FreeIMU
{
  public:
    FreeIMU();
    void init();
    void init(bool fastmode);
    void init(bool fastmode, int period);
    void init(int acc_addr, int gyro_addr, bool fastmode, int period);
    void getRawValues(int * raw_values);
    void getValues(float * values);
    void getQ(float * q);
    void getEuler(float * angles);
    void getYawPitchRoll(float * ypr);
    // we make them public so that users can interact directly with device classes
    HMC58X3 magn;
    ADXL345 acc;
    ITG3200 gyro;
    
  private:
    int* raw_acc, raw_gyro, raw_magn;
    void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    float q0, q1, q2, q3; // quaternion elements representing the estimated orientation
    float exInt, eyInt, ezInt;  // scaled integral error
    int period, lastUpdate, now; // sample period expressed in milliseconds
    float halfT; // half the sample period expressed in seconds
    int startLoopTime;
};

float invSqrt(float number);

#endif // FreeIMU_h


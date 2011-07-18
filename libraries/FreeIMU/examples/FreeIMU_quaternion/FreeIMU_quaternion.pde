#include <ADXL345.h>
#include <HMC58X3.h>
#include <ITG3200.h>

#define DEBUG
#ifdef DEBUG
#include "DebugUtils.h"
#endif

#include "CommunicationUtils.h"
#include "FreeIMU.h"
#include <Wire.h>


float q[4];

// Set the FreeIMU object
FreeIMU my3IMU = FreeIMU();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  delay(5);
  my3IMU.init();
  delay(5);
}


void loop() { 
  //my3IMU.startLoop();
  
  my3IMU.getQ(q);
  serialPrintFloatArr(q, 4);
  Serial.println("");
  delay(10);
  //my3IMU.endLoop();
}


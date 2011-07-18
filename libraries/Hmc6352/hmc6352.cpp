/*
 * hmc6352.cpp
 * 
 * Copyright (c) 2009 Ruben Laguna <ruben.laguna at gmail.com>. All rights reserved.
 * 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "hmc6352.h"
#include <Wire.h>

#include "WProgram.h"


Hmc6352::Hmc6352()
{
  Wire.begin();
}

Hmc6352 hmc6352=Hmc6352();


float Hmc6352::getHeading()
{
  byte val = 0;
  byte data[2];
  int  j, frac;

  Wire.beginTransmission(0x21);
  Wire.send(0x41); //A
  Wire.endTransmission();
  delay(8); //6000 microseconds minimum 6 ms 

  Wire.requestFrom(0x21, 2);
  j = 0;
  while(Wire.available())
  {
    char c = Wire.receive();
    data[j] = c;
    j++;
  }
  frac = data[0]*256 + data[1];
  
  return (frac/10.0);
}


void Hmc6352::wake()
{
  Wire.beginTransmission(0x21);
  Wire.send(0x57); //W wake up exit sleep mode
  Wire.endTransmission();
  
}

void Hmc6352::sleep()
{
  Wire.beginTransmission(0x21);
  Wire.send(0x53); //S enter sleep mode
  Wire.endTransmission();
  
}


/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include "vl53l1_register_map.h"

#include <Wire.h>

void setup(void)
{
  Wire.begin();

  Serial.begin(9600);
  Serial.println("VL53L1X Qwiic Test");

  if (begin() == false)
    Serial.println("Sensor offline!");

}

void loop(void)
{
  writeConfiguration(); //Write configuration block of 135 bytes to setup a measurement

  //Poll for completion of measurement. Takes 40-50ms.
  while (newDataReady() == false)
    delay(5);

  int distance = getDistance(); //Get the result of the measurement from the sensor

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  float distanceInches = distance * 0.0393701;
  float distanceFeet = distanceInches / 12.0;

  Serial.print("\tDistance(ft): ");
  Serial.print(distanceFeet, 2);

  Serial.println();
}


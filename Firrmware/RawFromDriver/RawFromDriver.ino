/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 21st, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14641

  This example prints which button was pressed. Press * for space and # for new line.
  
  Qwiic KeyPad records any button presses to a stack. If quered by the master KeyPad will
  respond with the oldest button pressed along with the time since it was pressed.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

byte keypadAddress = 75; //75 (0x4B) is default, 74 if jumper is opened

void setup(void)
{
  Wire.begin();
  
  Serial.begin(9600);
  Serial.println("Read Qwiic KeyPad Example");
  Serial.println("Press a button: * to do a space. # to go to next line.");
}

void loop(void)
{
  char button = readKeyPad();

  if(button == -1)
  {
    Serial.println("No keypad detected");
    delay(1000);
  }
  else if(button != 0)
  {
    if(button == '#') Serial.println();
    else if(button == '*') Serial.print(" ");
    else Serial.print(button);
  }

  //Do something else. Don't call readKeyPad a ton otherwise you'll tie up the I2C bus
  delay(25); //25 is good, more is better
}

//_register_map for all the registers
//_platform for the basic read/write byte and region I2C stuff
//_api for the WaitDeviceBooted, etc

//_wait is needed for waitdevicebooted - can be written out


void AutonomousLowPowerRangingTest(void)
{
  static VL53L1_RangingMeasurementData_t RangingData;

  printf("Autonomous Ranging Test\n");

  /**
   * Polls the bit 0 of the FIRMWARE__SYSTEM_STATUS register to see if
   * the firmware is ready.
   */  
  /* after reset for the firmware blocks I2C access while
   * it copies the NVM data into the G02 host register banks
   * The host must wait the required time to allow the copy
   * to complete before attempting to read the firmware status
   */
  status = VL53L1_WaitDeviceBooted(Dev);

  //Set pads to 2.8V
  
  status = VL53L1_DataInit(Dev);

  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 500);
  status = VL53L1_StartMeasurement(Dev);

  if(status){
    printf("VL53L1_StartMeasurement failed \n");
    while(1);
  } 
  if (isInterrupt){
    do // interrupt mode
    {
     __WFI();
     if(IntCount !=0 ){
        IntCount=0;
        status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
        if(status==0){
          printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
                  RangingData.SignalRateRtnMegaCps/65536.0,RangingData.AmbientRateRtnMegaCps/65336.0);
        }
        status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
      }
    }
    while(1);
  }
  else{
    do // polling mode
    {
      status = VL53L1_WaitMeasurementDataReady(Dev);
      if(!status)
      {
        status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
        if(status==0){
          printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
                  (RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
        }
        status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
      }
    }
    while (1);
  }
//  return status;
}

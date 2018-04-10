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
#include "vl53l1_register_map.h"

byte deviceAddress = 0x29; //Default 7-bit unshifted address of the VL53L1X

void setup(void)
{
  Wire.begin();

  Serial.begin(9600);
  Serial.println("VL53L1X Qwiic Test");

  unsigned int result = readRegister16(0x010F);
  Serial.print("Model_ID (Should be 0xEACC): 0x");
  Serial.println(result, HEX);

  //From I2C trace:

  //Check system status. Passing was 0x03.
  result = readRegister16(VL53L1_FIRMWARE__SYSTEM_STATUS);
  Serial.print("System status: 0x");
  Serial.println(result, HEX);
  
  //Then write 138 bytes in sequential order, starting from 0x00? No address given
  Write 0x 00 00 01 01 40 to do a times measurement

  //Then polling 0x0031 to become 0x02
  //Checking for interrupt to trigger?
  //Takes 45.46ms
  result = readRegister16(VL53L1_GPIO__TIO_HV_STATUS);
  Serial.print("New data ready: 0x");
  Serial.println(result, HEX);

  //Once we have 0x02...

  //Then read from 0x0088, 135 sequential bytes
  //Byte 14 and 15 should be distance
  14 = 0x00
  15 = 0x15
  = 21mm?

  //Then Write at 0x00?, 139 bytes

  //More polling, 51ms

  //Then read from 0x0088, 135 sequential bytes
  14 = 0x00
  15 = 0x11

  //Then Write at 0x00?, another 70 bytes

  AutonomousLowPowerRangingTest();
}

void loop(void)
{
}

//_register_map for all the registers
//_platform for the basic read/write byte and region I2C stuff
//_api for the WaitDeviceBooted, etc

//_wait is needed for waitdevicebooted - can be written out


void AutonomousLowPowerRangingTest(void)
{
  /**
     Polls the bit 0 of the FIRMWARE__SYSTEM_STATUS register to see if
     the firmware is ready.
  */
  /* after reset for the firmware blocks I2C access while
     it copies the NVM data into the G02 host register banks
     The host must wait the required time to allow the copy
     to complete before attempting to read the firmware status
  */
  //status = VL53L1_WaitDeviceBooted(Dev); VL53L1_WaitDeviceBooted
  /*VL53L1_WaitValueMaskEx(
    VL53L1_FIRMWARE__SYSTEM_STATUS, //0x00E5
    0x01, //Value
    0x01, //Mask
    VL53L1_POLLING_DELAY_MS); //VL53L1_POLLING_DELAY_MS = 1
  */

  //Polls the bit 0 of the FIRMWARE__SYSTEM_STATUS register to see if the firmware is ready.
  int counter = 0;
  while (counter++ < 100)
  {
    uint16_t result = readRegister16(VL53L1_FIRMWARE__SYSTEM_STATUS);
    if (result & 0x01 == 0)
    {
      Serial.print("Firmware ready");
      break;
    }
  }

  //status = VL53L1_DataInit(Dev);
  //Set I2C to 2.8V mode
  uint16_t result = readRegister16(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG);
  result = (result & 0xFE) | 0x01;
  writeRegister16(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG, result);


  //status = VL53L1_StaticInit(Dev);
  //  Status = VL53L1_SetPresetMode(Dev, VL53L1_PRESETMODE_LOWPOWER_AUTONOMOUS);
  //Status = SetPresetMode(Dev, PresetMode, DistanceMode, 1000);
    //Status = ComputeDevicePresetMode(PresetMode, DistanceMode, &device_preset_mode);
      //Does nothing but setup vars
    //Status =  VL53L1_get_preset_mode_timing_cfg(Dev, device_preset_mode, &dss_config__target_total_rate_mcps,   &phasecal_config_timeout_us,
        //&mm_config_timeout_us,
        //&lld_range_config_timeout_us);
      //Setup vars
    //Status = VL53L1_set_preset_mode(Dev, device_preset_mode, dss_config__target_total_rate_mcps, phasecal_config_timeout_us, mm_config_timeout_us,
        //lld_range_config_timeout_us,
        //inter_measurement_period_ms);
      //VL53L1_init_ll_driver_state(Dev, VL53L1_DEVICESTATE_SW_STANDBY);
        //Inits state vars
      //status = VL53L1_preset_mode_standard_ranging(pstatic, pgeneral, ptiming, pdynamic, psystem, ptuning_parms);

}

//Reads two consecutive bytes from a given location
uint16_t readRegister16(uint16_t addr)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr >> 8); //MSB
  Wire.write(addr & 0xFF); //LSB
  if (Wire.endTransmission(false) != 0) //Send a restart command. Do not release bus.
  {
    //Sensor did not ACK
    Serial.println("Error: Sensor did not ack");
  }

  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)2);
  if (Wire.available())
  {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return ((uint16_t)msb << 8 | lsb);
  }

  Serial.println("Error: Sensor did not respond");
  return (0);
}

//Write two bytes to a spot
void writeRegister16(uint16_t addr, uint16_t val)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr >> 8); //MSB
  Wire.write(addr & 0xFF); //LSB
  Wire.write(val >> 8); //MSB
  Wire.write(val & 0xFF); //LSB
  if (Wire.endTransmission() != 0)
  {
    //Sensor did not ACK
    Serial.println("Error: Sensor did not ack");
  }
}

/*

*/

byte deviceAddress = 0x29; //Default 7-bit unshifted address of the VL53L1X

const PROGMEM uint8_t configBlock[] = {
  0x29, 0x02, 0x10, 0x00, 0x28, 0xBC, 0x7A, 0x81, //8
  0x80, 0x07, 0x95, 0x00, 0xED, 0xFF, 0xF7, 0xFD, //16
  0x9E, 0x0E, 0x00, 0x10, 0x01, 0x00, 0x00, 0x00, //24
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x00, //32
  0x28, 0x00, 0x0D, 0x0A, 0x00, 0x00, 0x00, 0x00, //40
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, //48
  0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, //56
  0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x02, //64
  0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, //72
  0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x02, 0x00, //80
  0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, //88
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91, 0x0F, //96
  0x00, 0xA5, 0x0D, 0x00, 0x80, 0x00, 0x0C, 0x08, //104
  0xB8, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x10, 0x00, //112
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, //120
  0x0D, 0x0E, 0x0E, 0x01, 0x00, 0x02, 0xC7, 0xFF, //128
  0x8B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40 //129 - 135 (0x81 - 0x87)
};

//Check to see if sensor is responding
//Set sensor up for 2.8/3.3V I2C
boolean begin()
{
  //Check the device ID
  uint16_t modelID = readRegister16(VL53L1_IDENTIFICATION__MODEL_ID);
  if (modelID != 0xEACC) return (false);

  softReset();

  //Polls the bit 0 of the FIRMWARE__SYSTEM_STATUS register to see if the firmware is ready
  int counter = 0;
  while (readRegister16(VL53L1_FIRMWARE__SYSTEM_STATUS) & 0x01 == 0)
  {
    if (counter++ == 100) return (false); //Sensor timed out
    delay(10);
  }

  //Set I2C to 2.8V mode. In this mode 3.3V I2C is allowed.
  uint16_t result = readRegister16(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG);
  result = (result & 0xFE) | 0x01;
  writeRegister16(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG, result);

  return (true); //Sensor online!
}

//Write the 131 bytes of configuration, 32 bytes at a time
//This is the main function that setups up the VL53L1X to take measurements
//This was obtained by inspecting the example software from ST and by
//capturing I2C trace on ST Nucleo demo board
void writeConfiguration()
{
  uint8_t offset = 0; //Start at a location within the configBlock array
  uint8_t address = 1 + offset; //Start at memory location 0x01, add offset

  uint8_t leftToSend = sizeof(configBlock) - offset;
  while (leftToSend > 0)
  {
    uint8_t toSend = 30; //Max I2C buffer on Arduino is 32, and we need 2 bytes for address
    if (toSend > leftToSend) toSend = leftToSend;

    Wire.beginTransmission(deviceAddress);

    Wire.write(0); //We're only in lower address space. No MSB needed.
    Wire.write(address);

    for (byte x = 0 ; x < leftToSend ; x++)
      Wire.write(pgm_read_byte_near(configBlock + address + x - 1 - offset));

    Wire.endTransmission();

    leftToSend -= toSend;
    address += toSend;
  }
}

//Polls the measurement completion bit
boolean newDataReady()
{
  if (readRegister(VL53L1_GPIO__TIO_HV_STATUS) != 0x03) return(true); //New measurement!
  return(false); //No new data
}

//Reset sensor via software
void softReset()
{
  writeRegister(VL53L1_SOFT_RESET, 0x00); //Reset
  delay(1); //Driver uses 100us
  writeRegister(VL53L1_SOFT_RESET, 0x01); //Exit reset
}

//The sensor responds with 44 bytes of various datums.
//See VL53L1_i2c_decode_system_results() in vl53l1_register_funcs.c for the decoder ringer
//Start from memory address VL53L1_RESULT__INTERRUPT_STATUS
//0: result__interrupt_status,
//1: result__range_status, VL53L1_RESULT__RANGE_STATUS
//2: result__report_status, VL53L1_RESULT__REPORT_STATUS
//3: result__stream_count
//4: result__dss_actual_effective_spads_sd0
//6: result__peak_signal_count_rate_mcps_sd0
//8: result__ambient_count_rate_mcps_sd0
//10: result__sigma_sd0
//12: result__phase_sd0
//14: result__final_crosstalk_corrected_range_mm_sd0
//16: result__peak_signal_count_rate_crosstalk_corrected_mcps_sd0
//18: result__mm_inner_actual_effective_spads_sd0
//20: result__mm_outer_actual_effective_spads_sd0
//22: result__avg_signal_count_rate_mcps_sd0
//24: result__dss_actual_effective_spads_sd1
//26: result__peak_signal_count_rate_mcps_sd1
//28: result__ambient_count_rate_mcps_sd1
//30: result__sigma_sd1
//32: result__phase_sd1
//34: result__final_crosstalk_corrected_range_mm_sd1
//36: result__spare_0_sd1
//38: result__spare_1_sd1
//40: result__spare_2_sd1
//42: result__spare_3_sd1
//44: result__thresh_info

//Get the 'final' results from measurement
uint16_t getDistance()
{
  return (readRegister16(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0));
}

//Get signal rate
uint16_t getSignalRate()
{
  //From vl53l1_api.c line 2041
  uint16_t reading = readRegister16(VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);// << 9; //FIXPOINT97TOFIXPOINT1616
  //float signalRate = (float)reading/65536.0;
  return (reading);
}

//The sensor returns a range status that needs to be re-mapped to one of 9 different statuses
//This does that.
uint8_t getRangeStatus()
{
#define VL53L1_DEVICEERROR_VCSELCONTINUITYTESTFAILURE  ( 1)
#define VL53L1_DEVICEERROR_VCSELWATCHDOGTESTFAILURE    ( 2)
#define VL53L1_DEVICEERROR_NOVHVVALUEFOUND             ( 3)
#define VL53L1_DEVICEERROR_MSRCNOTARGET                ( 4)
#define VL53L1_DEVICEERROR_RANGEPHASECHECK             ( 5)
#define VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK         ( 6)
#define VL53L1_DEVICEERROR_PHASECONSISTENCY            ( 7)
#define VL53L1_DEVICEERROR_MINCLIP                     ( 8)
#define VL53L1_DEVICEERROR_RANGECOMPLETE               ( 9)
#define VL53L1_DEVICEERROR_ALGOUNDERFLOW               ( 10)
#define VL53L1_DEVICEERROR_ALGOOVERFLOW                ( 11)
#define VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD        ( 12)
#define VL53L1_DEVICEERROR_USERROICLIP                 ( 13)
#define VL53L1_DEVICEERROR_REFSPADCHARNOTENOUGHDPADS   ( 14)
#define VL53L1_DEVICEERROR_REFSPADCHARMORETHANTARGET   ( 15)
#define VL53L1_DEVICEERROR_REFSPADCHARLESSTHANTARGET   ( 16)
#define VL53L1_DEVICEERROR_MULTCLIPFAIL                ( 17)
#define VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY        ( 18)
#define VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK ( 19)
#define VL53L1_DEVICEERROR_EVENTCONSISTENCY            ( 20)
#define VL53L1_DEVICEERROR_MINSIGNALEVENTCHECK         ( 21)
#define VL53L1_DEVICEERROR_RANGECOMPLETE_MERGED_PULSE  ( 22)

#define VL53L1_RANGESTATUS_RANGE_VALID       0 /*!<The Range is valid. */
#define VL53L1_RANGESTATUS_SIGMA_FAIL        1 /*!<Sigma Fail. */
#define VL53L1_RANGESTATUS_SIGNAL_FAIL       2 /*!<Signal fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED 3 /*!<Target is below minimum detection threshold. */
#define VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL      4 /*!<Phase out of valid limits -  different to a wrap exit. */
#define VL53L1_RANGESTATUS_HARDWARE_FAIL     5 /*!<Hardware fail. */
#define VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL  6 /*!<The Range is valid but the wraparound check has not been done. */
#define VL53L1_RANGESTATUS_WRAP_TARGET_FAIL     7 /*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define VL53L1_RANGESTATUS_PROCESSING_FAIL      8 /*!<Internal algo underflow or overflow in lite ranging. */
#define VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL      9 /*!<Specific to lite ranging. */
#define VL53L1_RANGESTATUS_SYNCRONISATION_INT     10 /*!<1st interrupt when starting ranging in back to back mode. Ignore data. */
#define VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE   11 /*!<All Range ok but object is result of multiple pulses merging together.*/
#define VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL  12 /*!<Used  by RQL  as different to phase fail. */
#define VL53L1_RANGESTATUS_MIN_RANGE_FAIL     13 /*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define VL53L1_RANGESTATUS_RANGE_INVALID      14 /*!<lld returned valid range but negative value ! */
#define VL53L1_RANGESTATUS_NONE        255 /*!<No Update. */

  //Read status
  uint8_t measurementStatus = readRegister(VL53L1_RESULT__RANGE_STATUS) & 0x1F;

  //Convert status from one to another - From vl53l1_api.c
  switch (measurementStatus) {
    case VL53L1_DEVICEERROR_GPHSTREAMCOUNT0READY:
      measurementStatus = VL53L1_RANGESTATUS_SYNCRONISATION_INT;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE_NO_WRAP_CHECK:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGEPHASECHECK:
      measurementStatus = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
      break;
    case VL53L1_DEVICEERROR_MSRCNOTARGET:
      measurementStatus = VL53L1_RANGESTATUS_SIGNAL_FAIL;
      break;
    case VL53L1_DEVICEERROR_SIGMATHRESHOLDCHECK:
      measurementStatus = VL53L1_RANGESTATUS_SIGMA_FAIL;
      break;
    case VL53L1_DEVICEERROR_PHASECONSISTENCY:
      measurementStatus = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
      break;
    case VL53L1_DEVICEERROR_RANGEIGNORETHRESHOLD:
      measurementStatus = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
      break;
    case VL53L1_DEVICEERROR_MINCLIP:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
      break;
    case VL53L1_DEVICEERROR_RANGECOMPLETE:
      measurementStatus = VL53L1_RANGESTATUS_RANGE_VALID;
      break;
    default:
      measurementStatus = VL53L1_RANGESTATUS_NONE;
  }

  return measurementStatus;
}

//Reads two consecutive bytes from a given location
//Returns zero on error
uint16_t readRegister16(uint16_t addr)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr >> 8); //MSB
  Wire.write(addr & 0xFF); //LSB
  if (Wire.endTransmission() != 0) //Send a restart command. Do not release bus.
    return (0); //Sensor did not ACK

  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)2);
  if (Wire.available())
  {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return ((uint16_t)msb << 8 | lsb);
  }

  return (0); //Error: Sensor did not respond
}

//Reads one byte from a given location
//Returns zero on error
uint8_t readRegister(uint16_t addr)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr >> 8); //MSB
  Wire.write(addr & 0xFF); //LSB
  if (Wire.endTransmission() != 0) //Send a restart command. Do not release bus.
    return (0); //Sensor did not ACK

  Wire.requestFrom((uint8_t)deviceAddress, (uint8_t)1);
  if (Wire.available())
    return (Wire.read());

  return (0); //Error: Sensor did not respond
}

//Write a byte to a spot
boolean writeRegister(uint16_t addr, uint8_t val)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr >> 8); //MSB
  Wire.write(addr & 0xFF); //LSB
  Wire.write(val);
  if (Wire.endTransmission() != 0)
    return (0); //Error: Sensor did not ACK
  return (1); //All done!
}

//Write two bytes to a spot
boolean writeRegister16(uint16_t addr, uint16_t val)
{
  Wire.beginTransmission(deviceAddress);
  Wire.write(addr >> 8); //MSB
  Wire.write(addr & 0xFF); //LSB
  Wire.write(val >> 8); //MSB
  Wire.write(val & 0xFF); //LSB
  if (Wire.endTransmission() != 0)
    return (0); //Error: Sensor did not ACK
  return (1); //All done!
}

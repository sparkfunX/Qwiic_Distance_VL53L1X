
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


//Reset sensor via software
void softReset()
{
  writeRegister(VL53L1_SOFT_RESET, 0x00); //Reset
  delay(1); //Driver uses 100us
  writeRegister(VL53L1_SOFT_RESET, 0x01); //Exit reset
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




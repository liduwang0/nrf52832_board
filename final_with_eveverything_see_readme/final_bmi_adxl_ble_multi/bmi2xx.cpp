

extern "C" {
#include <string.h>
}
#include <SPI.h>
#include "bmi2xx.h"



unsigned int BMI2xx::readRegister(byte thisRegister, int bytesToRead) {
  byte inByte = 0;           // incoming byte from the SPI
  unsigned int result = 0;   // result to return
  byte dataToSend = thisRegister | READ;
  // take the chip select low to select the device:
  
  digitalWrite(SS, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  bytesToRead--;
  // if you still have another byte to read:
  while (bytesToRead > 0) {
    // shift the first byte left, then get the second byte:
    result = result << 8;
    inByte = SPI.transfer(0x00);
    // combine the byte you just got with the previous one:
    result = result | inByte;
    // decrement the number of bytes left to read:
    bytesToRead--;
  }
  // take the chip select high to de-select:
  digitalWrite(SS, HIGH);
  delay(1);
  // return the result:  
  return (result);
}

void BMI2xx::writeRegister(byte thisRegister, byte thisValue) {

  // now combine the register address and the command into one byte:
  byte dataToSend = thisRegister & WRITE;

  // take the chip select low to select the device:
  digitalWrite(SS, LOW);

  SPI.transfer(dataToSend); //Send register location
  SPI.transfer(thisValue);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(SS, HIGH);
  delay(1);
}

void BMI2xx::customized_mode_bmi(uint8_t PWR_control, uint8_t ACC_config, uint8_t GYR_config, uint8_t ACC_range_config, uint8_t GYR_range_config) {


SPI.setDataMode(SPI_MODE3);
  delay(1);


  writeRegister(PWR_CTRL,PWR_control); //00000110 temp off, acc on, gyr on, aux off

  writeRegister(ACC_CONF,ACC_config); //0 000 0111  power optimized, no averaging, 25Hz

  writeRegister(GYR_CONF,GYR_config); //0 0 00 0110  power optimized, power optimized, no averaging, 25Hz

  writeRegister(0x41,ACC_range_config); // acc range

  writeRegister(0x43,GYR_range_config); // gyr range
  
  writeRegister(PWR_CONF,0x03); // 00000 011  fast power up disable, FIFO read enable, power save enable. 默认
  
  Serial.println("BMI270 mode write set up finished...");

  delay(1);
}

void BMI2xx::Upload_file(int config_size_, int file_count_, byte* filepos_)
{
	byte dataToSend = INIT_ADDR_0 & WRITE;

    // take the chip select low to select the device:
    digitalWrite(SS, LOW);  
    SPI.transfer(dataToSend); //Send register location
    SPI.transfer(0x00);  //Send value to record into register
    SPI.transfer(file_count_);  //Send value to record into register
  
    // take the chip select high to de-select:
    digitalWrite(SS, HIGH);

    dataToSend = INIT_DATA & WRITE;
    digitalWrite(SS, LOW);
    SPI.transfer(dataToSend);
    SPI.transfer(filepos_,32);
    // take the chip select high to de-select:
    digitalWrite(SS, HIGH);
    delay(5);
}


void BMI2xx::Upload_ConfigFile(byte thisRegister,int config_size_, byte* filepos_)
{
  byte dataToSend = thisRegister & WRITE;
  
    digitalWrite(SS, LOW);
    SPI.transfer(dataToSend);
    SPI.transfer(filepos_,config_size_);
    digitalWrite(SS, HIGH);
    delay(1);
}

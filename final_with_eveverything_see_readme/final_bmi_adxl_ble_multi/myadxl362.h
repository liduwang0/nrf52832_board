

#include "Arduino.h"

#ifndef MYADXL362_h
#define MYADXL362_h

class MYADXL362
{
public:

	void Initialization(int16_t chipSelectPin); 
	byte SPIreadOneRegister(byte regAddress);
	void SPIwriteOneRegister(byte regAddress, byte regValue);
  void customized_mode_adxl(uint8_t pwr_control, uint8_t Hz_config);
private:

	
};

#endif
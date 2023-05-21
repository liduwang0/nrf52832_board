#include "Arduino.h"
#include "myadxl362.h"
#include <SPI.h>

int16_t slaveSelectPin = 15;

void MYADXL362::Initialization(int16_t chipSelectPin) {
  while (1) {      // 这个芯片有问题，经常初始化失败。必须循环电源启动，妈的。
    Serial.println("Begin to initialize adxl...  ");

    slaveSelectPin = chipSelectPin;  //初始化需要指定cspin
    pinMode(slaveSelectPin, OUTPUT);
    delay(10);

    pinMode(22, OUTPUT);  //打开IC开关
    digitalWrite(22, HIGH);

    pinMode(6, INPUT_PULLUP);  // 由于忘了上拉电阻，所以我需要给 mosi miso sck指定使用上拉电阻，最终版可以去掉
    pinMode(7, INPUT_PULLUP);
    pinMode(8, INPUT_PULLUP);


      SPI.begin();
      SPI.setDataMode(SPI_MODE0);  
    delay(1);
    SPIwriteOneRegister(0x1F, 0x52);  // Write to SOFT RESET, "R"
    delay(10);

    SPIwriteOneRegister(0x2D, 0x02);   //测量模式
    SPIwriteOneRegister(0x2C, 0x00);  //默认12.5 Hz
    delay(10);

    byte status_register = SPIreadOneRegister(0x2B);  //成功为0，否则255
    Serial.print("status: ");
    Serial.println(status_register);

    if (status_register == 0) {
      break;
    }
    if (status_register == 255) {
      pinMode(22, OUTPUT);
      digitalWrite(22, LOW);  // 这一坨都是必要的，不写678不行，试过了
      pinMode(6, OUTPUT);              // spi end不能代替这个，测试了，只有spi end一直初始化失败
      digitalWrite(6, LOW);
      pinMode(7, OUTPUT);                      
      digitalWrite(7, LOW);
      pinMode(8, OUTPUT);
      digitalWrite(8, LOW);
      // SPI.end(); // 这个就等同于关闭那些接口了
      //众多测试， 有操作678和spiend，也会初始化失败，。只有spi也会初始化失败，只能保留678才能初始化成功
      pinMode(slaveSelectPin, OUTPUT);
      digitalWrite(slaveSelectPin, LOW);
      delay(2000);  // 可能是电源不干净，断电之后要等电容放电，完全放电，至少2秒，试过了1秒不行， 1.5 也不行
    }
  }
}

void MYADXL362::SPIwriteOneRegister(byte regAddress, byte regValue) {

  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0A);  // write instruction
  SPI.transfer(regAddress);
  SPI.transfer(regValue);
  digitalWrite(slaveSelectPin, HIGH);
}

byte MYADXL362::SPIreadOneRegister(byte regAddress) {
  byte regValue = 0;

  digitalWrite(slaveSelectPin, LOW);
  SPI.transfer(0x0B);  // read instruction
  SPI.transfer(regAddress);
  regValue = SPI.transfer(0x00);
  digitalWrite(slaveSelectPin, HIGH);

  return regValue;
}

void MYADXL362::customized_mode_adxl(uint8_t pwr_control, uint8_t Hz_config) {
   SPI.setDataMode(SPI_MODE0);

   delay(1);

  SPIwriteOneRegister(0x1F, 0x52);  // Write to SOFT RESET, "R"
  delay(10);
  SPIwriteOneRegister(0x2D, pwr_control);  // off external clock, off wake up, off auto sleep, Measurement mode
  delay(10);
  SPIwriteOneRegister(0x2C, Hz_config);  // POWER CONTROL REGISTERAddress: 0x2D, Reset: 0x
  delay(10);
  Serial.println("adxl mode write set up finished...");

  delay(100);
}
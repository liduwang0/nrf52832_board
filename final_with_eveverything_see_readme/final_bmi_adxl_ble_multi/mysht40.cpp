#include "mysht40.h"
#include "Arduino.h"
#include <Wire.h>

uint8_t customized_mode = 0xFD;

void MYSHT40::customized_mode_sht40(uint8_t mode) {   
  Serial.println("sht40 mode write set up finished...");    // customized_mode 会被传递到get_data函数
  if (mode != 0x00) {
    // Wire.begin();
    pinMode(24, OUTPUT);
    digitalWrite(24, HIGH);
    pinMode(9,  INPUT_PULLUP);
    pinMode(10, INPUT_PULLUP);

    on_or_off_status = 1;
    customized_mode = mode;
  }

  if (mode == 0x00) { 
    // Wire.end();      //采集数据的程序就不要操作wire了，不必节省能量，因为会有bug，wire读取失败的时候会卡住，当正在采集数据的时候进行config device操作 会卡住，普通先stop在配置的话没问题
    pinMode(24, OUTPUT);
    digitalWrite(24, LOW);
    pinMode(9, OUTPUT);
    digitalWrite(9, LOW);
    pinMode(10, OUTPUT);
    digitalWrite(10, LOW);

    on_or_off_status = 0;
    customized_mode = 0x94;       // soft reset
    
  }
}

float* MYSHT40::get_data_float() {
  static float result[2] = { 0 };
  Wire.beginTransmission(0x44);
  Wire.write(customized_mode);
  Wire.endTransmission();
  delay(10);

  Wire.requestFrom(0x44, 6);
  byte rx_bytes[6];
  for (int i = 0; i < 6; i++) {
    rx_bytes[i] = Wire.read();
  }
  uint16_t t_ticks = rx_bytes[0] * 256 + rx_bytes[1];
  uint16_t checksum_t = rx_bytes[2];
  uint16_t rh_ticks = rx_bytes[3] * 256 + rx_bytes[4];
  uint16_t checksum_rh = rx_bytes[5];
  float t_degC = -45 + 175 * (float)t_ticks / 65535;
  float rh_pRH = -6 + 125 * (float)rh_ticks / 65535;
  if (rh_pRH > 100) {
    rh_pRH = 100;
  }
  if (rh_pRH < 0) {
    rh_pRH = 0;
  }
  result[0] = t_degC;
  result[1] = rh_pRH;
  return result;
}

byte* MYSHT40::get_data_byte() {
  static byte result[6] = { 0 };
  Wire.beginTransmission(0x44);
  Wire.write(customized_mode);
  Wire.endTransmission();
  delay(10);

  Wire.requestFrom(0x44, 6);
  byte rx_bytes[6];
  for (int i = 0; i < 6; i++) {
    rx_bytes[i] = Wire.read();
  }
  result[0] = rx_bytes[0];
  result[1] = rx_bytes[1];
  result[2] = rx_bytes[2];
  result[3] = rx_bytes[3];
  result[4] = rx_bytes[4];
  result[5] = rx_bytes[5];
  return result;
}
#ifndef MYSHT40_H
#define MYSHT40_H

#include "Arduino.h"


class MYSHT40 {
public:
  void customized_mode_sht40(uint8_t customized_mode);
  float* get_data_float(); //用于输出
  byte* get_data_byte(); //用于低功耗时，直接发送，不参与任何计算。
  byte on_or_off_status; //这个变量用于输出状态，用于控制我是否需要打开ic开关和关闭wire.end（）
private:
  
};

#endif

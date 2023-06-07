#ifndef MYCOMPRESS_H
#define MYCOMPRESS_H

#include "Arduino.h"


class MYCOMPRESS {
public:
      int8_t* scaleArray(int16_t arr[], int size);
      int8_t* squareRootAndScaleArray(int16_t arr[], int size);
private:
  
};

#endif

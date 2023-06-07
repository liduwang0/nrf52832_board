#include "mycompress.h"
#include "Arduino.h"

int8_t* MYCOMPRESS::scaleArray(int16_t arr[], int size) {
  static int8_t scaledArray[9];  // Assume size is always less than or equal to 100
  for (int i = 0; i < size; i++) {
    scaledArray[i] = arr[i] / 256;          // 在整数除法中，如果除法操作的结果不是一个整数，它会向下取整到最接近的整数。这意味着无论结果接近哪个整数，结果都会被向下取整到最接近的较小整数。因此，arr[i] / 256 的结果将被向下取整为一个整数值
  }
  return scaledArray;
}

int8_t* MYCOMPRESS::squareRootAndScaleArray(int16_t arr[], int size) {
  int8_t scaledArray[9];  // Assume size is always less than or equal to 100
  for (int i = 0; i < size; i++) {
    int16_t absVal = abs(arr[i]);
    float sqrtVal = sqrt(absVal);
    if (arr[i] < 0) {      // 如果原始值是负数
      sqrtVal = -sqrtVal;  // 将平方根变为负数
    }
    int8_t scaledVal = sqrtVal / 128;  // 缩放值以适应int8_t范围
    scaledArray[i] = scaledVal;  // 将缩放后的值存入数组
  }
  return scaledArray;
}
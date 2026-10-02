// Test-Ersatz für die LED-Matrix: merkt sich das zuletzt gezeigte Bild.
#pragma once
#include "Arduino.h"
#define renderBitmap(bitmap, rows, columns) loadPixels(&bitmap[0][0], rows * columns)
class Arduino_LED_Matrix {
 public:
  int begin() { return 1; }
  void loadPixels(uint8_t *arr, size_t size) { memcpy(pixels, arr, size < 104 ? size : 104); frames++; }
  uint8_t pixels[104] = {0};
  int frames = 0;
};

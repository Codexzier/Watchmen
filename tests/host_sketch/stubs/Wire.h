// Test-Ersatz für I2C mit einem simulierten MPU6050.
#pragma once
#include "Arduino.h"
class SimWire {
 public:
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t a) { addr = a; txBytes.clear(); }
  void write(uint8_t b) { txBytes.push_back(b); }
  uint8_t endTransmission() {
    if (addr != 0x68 || !present) return 2;
    if (!txBytes.empty()) reg = txBytes[0];
    return 0;
  }
  size_t requestFrom(uint8_t a, size_t n) {
    if (a != 0x68 || !present) return 0;
    rxBytes.clear();
    int16_t v[3] = {ax, ay, az};
    if (reg == 0x3B) for (int i = 0; i < 3; i++) { rxBytes.push_back((uint8_t)((uint16_t)v[i] >> 8)); rxBytes.push_back((uint8_t)(v[i] & 0xFF)); }
    while (rxBytes.size() < n) rxBytes.push_back(0);
    pos = 0;
    return n;
  }
  int read() { return pos < rxBytes.size() ? rxBytes[pos++] : -1; }
  bool present = true;
  int16_t ax = 0, ay = 0, az = 16384;
  uint8_t addr = 0, reg = 0;
  std::vector<uint8_t> txBytes, rxBytes;
  size_t pos = 0;
};
extern SimWire Wire;

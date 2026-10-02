// Test-Ersatz für SPI: merkt sich den zuletzt gesendeten Puffer.
#pragma once
#include "Arduino.h"
struct SPISettings {
  SPISettings(uint32_t c, int, int) : clock(c) {}
  uint32_t clock;
};
class SimSPI {
 public:
  void begin() {}
  void beginTransaction(SPISettings s) { clock = s.clock; }
  void transfer(void *buf, size_t n) { last.assign((uint8_t *)buf, (uint8_t *)buf + n); transfers++; }
  void endTransaction() {}
  std::vector<uint8_t> last;
  uint32_t clock = 0;
  int transfers = 0;
};
extern SimSPI SPI;

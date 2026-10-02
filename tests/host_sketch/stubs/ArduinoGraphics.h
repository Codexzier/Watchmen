// Test-Ersatz: Schriftart mit einfachem 3x5-Block je Zeichen (Leerzeichen = leer).
#pragma once
#include <cstdint>
struct Font {
  const int width;
  const int height;
  const uint8_t **data;
};
extern const struct Font Font_4x6;

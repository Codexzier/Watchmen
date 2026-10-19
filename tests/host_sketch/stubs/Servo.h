// Test-Ersatz für die Servo-Bibliothek: merkt sich die letzte Impulslänge.
#pragma once
class Servo {
 public:
  unsigned char attach(int p, int mn, int mx) { pin = p; minUs = mn; maxUs = mx; attached = true; return 0; }
  void detach() { attached = false; }
  void writeMicroseconds(int us) { lastUs = us; }
  int pin = -1, minUs = 0, maxUs = 0, lastUs = 0;
  bool attached = false;
};

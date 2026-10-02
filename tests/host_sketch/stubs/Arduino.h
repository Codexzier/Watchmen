// Minimaler Ersatz für Arduino.h, damit sketch.ino auf dem PC getestet werden kann.
// Nur für Tests! Auf dem UNO Q wird die echte Arduino-Umgebung verwendet.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>
#include <deque>

typedef uint8_t byte;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define A0 14
#define MSBFIRST 1
#define SPI_MODE0 0

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
int digitalRead(int pin);
int analogRead(int pin);
void analogReadResolution(int bits);

template <class T, class L, class H>
auto constrain(const T &amt, const L &low, const H &high) -> decltype(amt < low ? low : (amt > high ? high : amt)) {
  return amt < low ? low : (amt > high ? high : amt);
}

class String {
 public:
  String(const char *c = "") : s(c) {}
  String(const std::string &x) : s(x) {}
  unsigned int length() const { return (unsigned int)s.size(); }
  char charAt(unsigned int i) const { return s[i]; }
  const char *c_str() const { return s.c_str(); }
 private:
  std::string s;
};

class SimSerial {
 public:
  void begin(unsigned long b) { baud = b; }
  int available() { return (int)rx.size(); }
  int read() { if (rx.empty()) return -1; int v = rx.front(); rx.pop_front(); return v; }
  size_t write(const uint8_t *b, size_t n) { tx.insert(tx.end(), b, b + n); return n; }
  unsigned long baud = 0;
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
};
extern SimSerial Serial1;

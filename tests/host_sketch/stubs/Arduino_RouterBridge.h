// Test-Ersatz für die Bridge: zeichnet alle notify()-Aufrufe auf.
#pragma once
#include "Arduino.h"
#include <map>
#include <string>
#include <vector>

struct NotifyRecord {
  std::string name;
  std::vector<long> args;
};

class SimBridge {
 public:
  bool begin() { return true; }
  template <typename F>
  bool provide_safe(const char *name, F) { provided.push_back(name); return true; }
  template <typename... Args>
  void notify(const char *name, Args... args) {
    NotifyRecord r;
    r.name = name;
    (r.args.push_back((long)args), ...);
    log.push_back(r);
  }
  std::vector<std::string> provided;
  std::vector<NotifyRecord> log;
};
extern SimBridge Bridge;

class SimMonitor {
 public:
  bool begin(unsigned long = 0) { return true; }
  void println(const char *s);
};
extern SimMonitor Monitor;

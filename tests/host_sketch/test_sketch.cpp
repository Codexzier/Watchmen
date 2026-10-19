// Host-Test für sketch/sketch.ino.
// Baut den Sketch mit Ersatz-Headern (stubs/) für den PC und simuliert die Hardware:
// Zeit, Schrittmotor-Welle mit Kontaktschalter, A0, Radar-UART, MPU6050, LEDs, Matrix.
// Aufruf: siehe run_tests.sh
#include <cassert>
#include <cstdio>
#include <iostream>
#include "Arduino.h"
#include "Arduino_RouterBridge.h"
#include "SPI.h"
#include "Wire.h"

// ---------------- simulierte Hardware ----------------
static unsigned long long simUs = 0;            // simulierte Zeit in µs
static int pinState[64] = {0};                  // Ausgangszustände
static float a0Volts = 3.3f;                    // Spannung an A0
static long shaftPos = 0;                       // physikalische Wellenposition [Halbschritte]
static long switchAt = 0;                       // Schalter gedrückt, wenn shaftPos <= switchAt
static int lastCoilIndex = -1;                  // letzte erkannte Halbschritt-Phase
static long coilErrors = 0;                     // unzulässige Phasensprünge

static const uint8_t SIM_HALFSTEP[8] = {0b1000, 0b1100, 0b0100, 0b0110, 0b0010, 0b0011, 0b0001, 0b1001};

SimSerial Serial1;
SimBridge Bridge;
SimMonitor Monitor;
SimSPI SPI;
SimWire Wire;

void SimMonitor::println(const char *s) { std::printf("    [Monitor] %s\n", s); }

unsigned long millis() { return (unsigned long)(simUs / 1000ULL); }
unsigned long micros() { return (unsigned long)simUs; }
void delay(unsigned long ms) { simUs += ms * 1000ULL; }
void pinMode(int, int) {}
void analogReadResolution(int) {}
int analogRead(int pin) { return pin == A0 ? (int)(a0Volts / 3.3f * 4095.0f) : 0; }

static void coilsChanged() {
  uint8_t p = (pinState[4] << 3) | (pinState[7] << 2) | (pinState[8] << 1) | pinState[9];
  if (p == 0) return;                           // Spulen aus → Position bleibt
  int idx = -1;
  for (int i = 0; i < 8; i++) if (SIM_HALFSTEP[i] == p) idx = i;
  if (idx < 0) return;                          // Zwischenzustand beim Umschalten
  if (lastCoilIndex >= 0 && idx != lastCoilIndex) {
    int d = (idx - lastCoilIndex + 8) % 8;
    if (d == 1) shaftPos++;
    else if (d == 7) shaftPos--;
    else coilErrors++;
  }
  lastCoilIndex = idx;
}

void digitalWrite(int pin, int value) {
  pinState[pin] = value ? 1 : 0;
  if (pin == 9) coilsChanged();                 // IN4 wird als letztes geschrieben
}

int digitalRead(int pin) {
  if (pin == 2) return (shaftPos <= switchAt) ? 0 : 1; // Schalter gegen GND
  return pinState[pin];
}

// Font-Ersatz: jedes druckbare Zeichen ist ein 3x5-Block, Leerzeichen ist leer.
static const uint8_t BLOCK[6] = {0xE0, 0xA0, 0xA0, 0xA0, 0xE0, 0x00};
static const uint8_t EMPTY[6] = {0, 0, 0, 0, 0, 0};
static const uint8_t *GLYPHS[256];
#include "ArduinoGraphics.h"
const struct Font Font_4x6 = {4, 6, GLYPHS};

// ---------------- Sketch einbinden ----------------
#include "../../sketch/sketch.ino"

// ---------------- Hilfen ----------------
static void runFor(unsigned long ms) {
  unsigned long long end = simUs + ms * 1000ULL;
  while (simUs < end) {
    loop();
    simUs += 1100;                              // ca. 1,1 ms pro loop() (Bridge-Pause)
  }
}

static int countNotify(const char *name) {
  int n = 0;
  for (auto &r : Bridge.log) if (r.name == name) n++;
  return n;
}

static const NotifyRecord *lastNotify(const char *name) {
  for (auto it = Bridge.log.rbegin(); it != Bridge.log.rend(); ++it) if (it->name == name) return &*it;
  return nullptr;
}

static bool hasEvent(int code) {
  for (auto &r : Bridge.log) if (r.name == "mcu_event" && !r.args.empty() && r.args[0] == code) return true;
  return false;
}

static int failures = 0;
#define CHECK(cond, msg) do { if (cond) std::printf("  OK   %s\n", msg); else { std::printf("  FAIL %s (Zeile %d)\n", msg, __LINE__); failures++; } } while (0)

static std::vector<int> defaultConfig() {
  return {450, 1350, 900, 450, 1350, 900, -100, 1, 20, 3000, 0, 500, 90, 150, 500, 2500, 1, 5000, 64, 1};
}

int main() {
  for (int i = 0; i < 256; i++) GLYPHS[i] = (i == ' ') ? EMPTY : BLOCK;

  std::printf("Start / A0 / Konfiguration\n");
  shaftPos = 800; switchAt = 0;
  setup();
  CHECK(Bridge.provided.size() == 11, "11 RPC-Funktionen registriert");
  runFor(50);
  CHECK(motorsPresent, "A0 = 3,3 V -> Motoren erkannt");
  CHECK(hasEvent(EVT_MOTORS_ON), "Ereignis MOTORS_ON gemeldet");
  CHECK(!tiltAxis.attached, "Servos ohne Konfiguration noch aus");
  CHECK(rpcSetConfig(defaultConfig()), "Konfiguration angenommen");
  CHECK(tiltAxis.attached && radarAxis.attached, "Servos nach Konfiguration aktiv");
  CHECK(tiltAxis.servo.lastUs == 1500, "Servo 1 bei 90 Grad = 1500 us");
  runFor(300);
  const NotifyRecord *st = lastNotify("mcu_status");
  CHECK(st && st->args.size() == 15 && st->args[1] == 1, "Statusmeldung mit 15 Werten, Konfig ok");

  std::printf("Referenzfahrt\n");
  CHECK(!rpcPanTo(1000), "pan_to ohne Referenz wird abgelehnt");
  CHECK(rpcHome(), "Referenzfahrt gestartet");
  runFor(8000);
  CHECK(homingState == HOMING_DONE && homed, "Referenzfahrt erfolgreich");
  CHECK(hasEvent(EVT_HOMING_DONE), "Ereignis HOMING_DONE gemeldet");
  CHECK(stepPos == HOMING_BACKOFF, "Position nach Referenz = Rückzug (40)");
  long offset = shaftPos - stepPos;
  CHECK(std::labs(offset) <= 2, "Referenz-Versatz zur Schalterkante <= 2 Halbschritte");
  CHECK(coilErrors == 0, "keine unzulässigen Phasensprünge");

  std::printf("Fahren auf Position\n");
  CHECK(rpcPanTo(2000), "pan_to(2000) angenommen");
  unsigned long t0 = millis();
  while (stepPos != 2000 && millis() - t0 < 10000) runFor(10);
  unsigned long dt = millis() - t0;
  CHECK(stepPos == 2000 && shaftPos - offset == 2000, "Position 2000 erreicht (Welle stimmt)");
  std::printf("       Fahrzeit 1960 Halbschritte: %lu ms\n", dt);
  CHECK(dt > 3000 && dt < 6000, "Fahrzeit plausibel (Rampe, max. 500 Schritte/s)");
  runFor(600);
  CHECK(!coilsOn, "Spulen im Stillstand abgeschaltet");
  CHECK(rpcPanTo(99999) && stepTarget == 3000, "Ziel wird auf pan_max begrenzt");
  rpcStop();
  CHECK(stepTarget == stepPos, "stop hält sofort an");
  rpcPanTo(500);
  runFor(6000);
  CHECK(stepPos == 500, "zurück auf 500");

  std::printf("Schalter als Endanschlag beim Joggen\n");
  rpcPanJog(-2000);
  runFor(6000);
  CHECK(stepPos == 0 && !hasEvent(EVT_SWITCH_HIT), "Joggen mit Referenz wird bei 0 begrenzt");
  rpcPanTo(500);
  runFor(6000);
  switchAt += 30;                               // simulierter Schrittverlust: Schalter kommt 30 Schritte früher
  rpcPanJog(-2000);
  runFor(6000);
  CHECK(stepPos == 0 && hasEvent(EVT_SWITCH_HIT), "Schalter setzt Position bei Schrittverlust neu auf 0");
  switchAt -= 30;
  rpcHome();
  runFor(3000);
  CHECK(homed, "neue Referenzfahrt nach dem Test");

  std::printf("Servos und Radar-Ausgleich\n");
  rpcRadar(-1);
  CHECK(rpcTilt(1200, 0), "Neigung 120 Grad angefordert");
  runFor(1000);
  CHECK(std::fabs(tiltAxis.current - 120.0f) < 0.1f, "Servo 1 steht auf 120 Grad");
  CHECK(std::fabs(radarAxis.current - 60.0f) < 0.1f, "Servo 2 gleicht aus: 90 - (120-90) = 60 Grad");
  rpcTilt(1700, 0);
  runFor(1000);
  CHECK(std::fabs(tiltAxis.current - 135.0f) < 0.1f, "Neigung im Normalbetrieb auf Maximum 135 begrenzt");
  rpcTilt(1700, 1);
  runFor(1000);
  CHECK(std::fabs(tiltAxis.current - 170.0f) < 0.1f, "Kalibriermodus (raw) erlaubt 170 Grad");
  rpcRadar(300);
  runFor(1000);
  CHECK(std::fabs(radarAxis.current - 30.0f) < 0.1f, "Radar-Servo von Hand auf 30 Grad");
  rpcRadar(-1);
  rpcTilt(900, 0);
  runFor(1500);
  CHECK(std::fabs(radarAxis.current - 90.0f) < 0.1f, "Radar zurück auf Automatik (waagerecht 90)");

  std::printf("RD-03D Radar\n");
  CHECK(Serial1.baud == 256000, "UART mit 256000 Baud");
  bool cmdOk = Serial1.tx.size() >= 12 && Serial1.tx[0] == 0xFD && Serial1.tx[6] == 0x90;
  CHECK(cmdOk, "Multi-Ziel-Befehl gesendet");
  const uint8_t frame[30] = {0xAA, 0xFF, 0x03, 0x00, 0x0E, 0x03, 0xB1, 0x86, 0x10, 0x00, 0x68, 0x01,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x55, 0xCC};
  Serial1.rx.push_back(0x12);                   // Müllbyte vor dem Rahmen
  Serial1.rx.push_back(0xAA);                   // angefangener falscher Kopf
  for (uint8_t b : frame) Serial1.rx.push_back(b);
  runFor(200);
  const NotifyRecord *rr = lastNotify("mcu_radar");
  CHECK(rr && rr->args.size() == 9, "Radar-Meldung mit 9 Werten");
  CHECK(rr && rr->args[0] == -782 && rr->args[1] == 1713 && rr->args[2] == -16, "Ziel 1: x=-782 mm, y=1713 mm, v=-16 cm/s");
  CHECK(radarOk, "Radar als verfügbar markiert");
  runFor(2500);
  CHECK(!radarOk, "Radar nach 2 s ohne Daten als weg markiert");

  std::printf("WS2812B\n");
  rpcLed(0, 0xFF0000, 0);
  rpcLed(1, 0x0000FF, 0);
  runFor(100);
  CHECK(SPI.clock == 5000000, "SPI mit 5 MHz");
  CHECK(SPI.last.size() == 8 + 48 + 4, "60 Bytes (8 Vorlauf + 48 Daten + 4 Nachlauf)");
  uint8_t g1 = 0, r1 = 0, b2 = 0;
  for (int i = 0; i < 8; i++) {
    g1 = (g1 << 1) | (SPI.last[8 + i] == 0xF0);
    r1 = (r1 << 1) | (SPI.last[16 + i] == 0xF0);
    b2 = (b2 << 1) | (SPI.last[8 + 24 + 16 + i] == 0xF0);
  }
  CHECK(g1 == 0 && r1 == 64 && b2 == 64, "GRB-Reihenfolge und Helligkeit 64 korrekt");
  bool onlyValid = true;
  for (int i = 8; i < 56; i++) if (SPI.last[i] != 0xF0 && SPI.last[i] != 0xC0) onlyValid = false;
  CHECK(onlyValid, "nur 0xF0/0xC0 als Bitmuster");
  rpcLed(2, 0xFF8000, 500);
  int tr = SPI.transfers;
  runFor(2000);
  CHECK(SPI.transfers - tr >= 4 && SPI.transfers - tr <= 8, "Blinken im Sekundentakt erzeugt wenige Übertragungen");

  std::printf("LED-Matrix\n");
  rpcText("CAR");
  runFor(50);
  int lit = 0, firstCol = 99, lastCol = -1;
  for (int y = 0; y < 8; y++) for (int x = 0; x < 13; x++) if (matrix.pixels[y * 13 + x]) { lit++; if (x < firstCol) firstCol = x; if (x > lastCol) lastCol = x; }
  CHECK(!textScrolling, "CAR steht (kein Lauftext)");
  CHECK(firstCol == 1 && lastCol == 11, "CAR ist zentriert (Spalte 1..11)");
  CHECK(lit > 0 && matrix.pixels[0] == 0, "oberste Zeile frei (1 Pixel Abstand)");
  rpcText("HUMEN");
  int f0 = matrix.frames;
  runFor(1000);
  CHECK(textScrolling && matrix.frames - f0 >= 9, "HUMEN läuft als Lauftext");
  rpcText("");
  runFor(50);
  lit = 0;
  for (int i = 0; i < 104; i++) lit += matrix.pixels[i];
  CHECK(lit == 0, "leerer Text löscht die Matrix");

  std::printf("MPU6050 / Vibration / Lage\n");
  runFor(2000);
  CHECK(mpuOk, "MPU6050 erkannt");
  CHECK(std::fabs(accAvgZ - 1000.0f) < 30.0f, "Z-Achse ca. 1000 mg (normal montiert)");
  Wire.az = 16384 + 8000;                       // kurzer Stoß (~ +490 mg)
  runFor(10);
  Wire.az = 16384;
  runFor(100);
  CHECK(countNotify("mcu_vibration") == 1, "Vibration gemeldet");
  CHECK(flashCount == 3 && flashColor == 0x8000FF, "LEDs blitzen 3x lila");
  rpcPanTo(1500);
  runFor(200);
  Wire.az = 16384 + 8000;
  runFor(10);
  Wire.az = 16384;
  runFor(100);
  CHECK(countNotify("mcu_vibration") == 1, "keine Vibrationsmeldung während der Motor fährt");
  runFor(5000);
  Wire.az = -16384;                             // Deckenmontage
  runFor(3000);
  CHECK(accAvgZ < -900.0f, "Deckenmontage: Z-Achse negativ");

  std::printf("A0 fällt weg\n");
  a0Volts = 0.0f;
  runFor(400);
  CHECK(!motorsPresent && hasEvent(EVT_MOTORS_OFF), "Motoren entfernt erkannt");
  CHECK(!tiltAxis.attached && !coilsOn && !homed, "Servos/Spulen aus, Referenz verworfen");
  CHECK(!rpcPanTo(100) && !rpcTilt(900, 0), "Bewegungsbefehle werden abgelehnt");
  a0Volts = 3.3f;
  runFor(400);
  CHECK(motorsPresent && tiltAxis.attached, "Motoren wieder da, Servos wieder aktiv");

  std::printf("Referenzfahrt ohne Schalter\n");
  switchAt = -100000;                           // Schalter defekt
  rpcHome();
  runFor(25000);
  CHECK(homingState == HOMING_FAILED && hasEvent(EVT_HOMING_FAILED), "Fehler nach max. Suchweg");

  std::printf("\n%s (%d Fehler)\n", failures == 0 ? "ALLE TESTS BESTANDEN" : "TESTS FEHLGESCHLAGEN", failures);
  return failures == 0 ? 0 : 1;
}

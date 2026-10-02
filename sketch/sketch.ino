// =====================================================================================
//  Watchmen – Mikrocontroller-Teil (STM32U585 im Arduino UNO Q)
// =====================================================================================
//  Dieser Sketch läuft auf dem Mikrocontroller (MCU) des UNO Q und übernimmt alle
//  Aufgaben, die exaktes Timing brauchen:
//    - Servo 1 (Kamera-/Plattform-Neigung) und Servo 2 (Radar-Neigung, Lageausgleich)
//    - Schrittmotor 28BYJ-48 (über ULN2003) inkl. Referenzfahrt zum Kontaktschalter
//    - 2x WS2812B RGB-LEDs (Datenleitung über SPI-MOSI, damit das Timing stimmt)
//    - Text auf der eingebauten 13x8 LED-Matrix (z.B. "CAR", "HUMEN")
//    - AI-Thinker RD-03D Radar (UART, 256000 Baud, bis zu 3 Ziele)
//    - MPU6050 (I2C): Vibrationserkennung und Einbaulage (Deckenmontage)
//    - Pin A0: Erkennung, ob Servos/Schrittmotor angeschlossen sind (3,3 V = ja)
//  Die eigentliche Logik (Zustände, Kamera, Objekterkennung, Webseite) läuft auf der
//  Linux-Seite in python/. Beide Seiten reden über die "Bridge" (RPC) miteinander.
// =====================================================================================

#include <Arduino_RouterBridge.h>   // Bridge (RPC) zwischen MCU und Linux-Seite des UNO Q
#include <Arduino_LED_Matrix.h>     // Ansteuerung der eingebauten 13x8 LED-Matrix
#include <ArduinoGraphics.h>        // Enthält die kleine Schriftart Font_4x6 für die Matrix
#include <Servo.h>                  // Servo-Bibliothek (Version mit Zephyr-/UNO-Q-Unterstützung)
#include <SPI.h>                    // SPI-Schnittstelle, hier nur MOSI (D11) für die WS2812B
#include <Wire.h>                   // I2C-Schnittstelle für den MPU6050
#include <vector>                   // std::vector, damit Python Listen schicken kann
#include <math.h>                   // sqrtf() für Beträge und Bremsrampe

// ------------------------------- Pinbelegung -----------------------------------------
const int PIN_SWITCH      = 2;      // Kontaktschalter der Drehachse (schaltet gegen GND)
const int PIN_MPU_INT     = 3;      // INT-Ausgang des MPU6050 (reserviert für Erweiterungen)
const int PIN_STEP_IN1    = 4;      // ULN2003 Eingang IN1 (Schrittmotor)
const int PIN_SERVO_TILT  = 5;      // Servo 1: Neigung der Plattform mit Kamera
const int PIN_SERVO_RADAR = 6;      // Servo 2: Neigung des RD-03D (wird waagerecht gehalten)
const int PIN_STEP_IN2    = 7;      // ULN2003 Eingang IN2 (Schrittmotor)
const int PIN_STEP_IN3    = 8;      // ULN2003 Eingang IN3 (Schrittmotor)
const int PIN_STEP_IN4    = 9;      // ULN2003 Eingang IN4 (Schrittmotor)
// Hinweis: D10 (NSS), D12 (MISO) und D13 (SCK) gehören zum SPI und bleiben frei.
// Hinweis: D11 (SPI MOSI) geht über einen Pegelwandler an DIN der ersten WS2812B.
// Hinweis: D0/D1 (Serial1) gehen an das RD-03D, SDA/SCL (Wire) an den MPU6050.
const int PIN_MOTOR_SENSE = A0;     // 3,3 V an A0 bedeutet: Servos und Schrittmotor sind da

// ------------------------------- Zeit-Konstanten -------------------------------------
const int FW_VERSION                 = 1;     // Versionsnummer dieses Sketches
const unsigned long STATUS_MS        = 100;   // alle 100 ms Status an Linux melden
const unsigned long RADAR_NOTIFY_MS  = 100;   // Radar-Ziele höchstens alle 100 ms melden
const unsigned long RADAR_TIMEOUT_MS = 2000;  // ohne Radar-Daten für 2 s gilt Radar als weg
const unsigned long IMU_MS           = 5;     // MPU6050 alle 5 ms lesen (200 Hz)
const unsigned long SERVO_MS         = 20;    // Servos alle 20 ms nachführen (50 Hz)
const unsigned long LED_MS           = 20;    // LED-Zustand alle 20 ms berechnen
const unsigned long LED_REFRESH_MS   = 1000;  // LEDs spätestens jede Sekunde neu senden
const unsigned long FLASH_MS         = 150;   // Dauer einer Hell- bzw. Dunkelphase beim Blitzen
const unsigned long SCROLL_MS        = 90;    // Lauftext: alle 90 ms eine Spalte weiter
const unsigned long SENSE_MS         = 50;    // A0 alle 50 ms prüfen
const unsigned long COILS_OFF_MS     = 400;   // Schrittmotor-Spulen 400 ms nach Stillstand aus
const unsigned long QUIET_MS         = 700;   // Vibration erst 700 ms nach Motorbewegung werten
const unsigned long VIB_GAP_MS       = 1500;  // Mindestabstand zwischen zwei Vibrationsmeldungen

// ------------------------------- Schrittmotor-Konstanten -----------------------------
const float STEP_MIN_SPEED   = 150.0f;  // Start-/Mindestgeschwindigkeit [Halbschritte/s]
const float STEP_ACCEL       = 1200.0f; // Beschleunigung [Halbschritte/s²]
const float HOMING_SPEED     = 300.0f;  // Geschwindigkeit bei der Referenzfahrt
const long  HOMING_BACKOFF   = 40;      // nach dem Schalter so viele Schritte zurückfahren
const int   HOMING_IDLE      = 0;       // Referenzfahrt: nichts zu tun
const int   HOMING_SEARCH    = 1;       // Referenzfahrt: fahre Richtung Schalter
const int   HOMING_LEAVE     = 2;       // Referenzfahrt: fahre vom Schalter weg
const int   HOMING_DONE      = 3;       // Referenzfahrt: erfolgreich beendet
const int   HOMING_FAILED    = -1;      // Referenzfahrt: Schalter nicht gefunden

// Halbschritt-Folge für 28BYJ-48 (Bit3=IN1, Bit2=IN2, Bit1=IN3, Bit0=IN4)
const uint8_t HALFSTEP[8] = {0b1000, 0b1100, 0b0100, 0b0110, 0b0010, 0b0011, 0b0001, 0b1001};

// ------------------------------- Ereignis-Codes an Linux -----------------------------
const int EVT_HOMING_DONE    = 1;   // Referenzfahrt fertig
const int EVT_HOMING_FAILED  = 2;   // Referenzfahrt fehlgeschlagen
const int EVT_MOTORS_ON      = 3;   // Motoren wurden angeschlossen (A0 = 3,3 V)
const int EVT_MOTORS_OFF     = 4;   // Motoren wurden entfernt (A0 = 0 V)
const int EVT_CONFIG_OK      = 5;   // Konfiguration wurde übernommen
const int EVT_SWITCH_HIT     = 6;   // Schalter beim normalen Fahren getroffen (Position neu gesetzt)

// ------------------------------- Konfiguration (kommt von Linux) ---------------------
struct Config {                       // Alle einstellbaren Werte an einem Ort
  float tiltMin    = 45.0f;           // kleinster erlaubter Winkel Servo 1 [Grad]
  float tiltMax    = 135.0f;          // größter erlaubter Winkel Servo 1 [Grad]
  float tiltLevel  = 90.0f;           // Winkel Servo 1, bei dem die Kamera waagerecht ist
  float radarMin   = 45.0f;           // kleinster erlaubter Winkel Servo 2 [Grad]
  float radarMax   = 135.0f;          // größter erlaubter Winkel Servo 2 [Grad]
  float radarLevel = 90.0f;           // Winkel Servo 2, bei dem das Radar waagerecht ist
  float radarComp  = -1.0f;           // Ausgleichsfaktor: Radar = Level + Faktor * Neigung
  bool  radarCompOn = true;           // true = Radar wird automatisch ausgeglichen
  long  panMin     = 20;              // kleinste erlaubte Position Drehachse [Halbschritte]
  long  panMax     = 3000;            // größte erlaubte Position Drehachse [Halbschritte]
  bool  stepInvert = false;           // true = Spulenfolge umdrehen (Schalter liegt andersrum)
  float stepMaxSpeed = 500.0f;        // Höchstgeschwindigkeit Schrittmotor [Halbschritte/s]
  float servoSpeed = 90.0f;           // Höchstgeschwindigkeit Servos [Grad/s]
  int   vibThreshold = 150;           // Vibrationsschwelle [mg = tausendstel g]
  int   servoUsMin = 500;             // Impulslänge bei 0 Grad [Mikrosekunden]
  int   servoUsMax = 2500;            // Impulslänge bei 180 Grad [Mikrosekunden]
  bool  calibrated = false;           // true = Kalibrierung 1 abgeschlossen (Grenzen gelten)
  long  homingMaxSteps = 5000;        // so weit darf die Referenzfahrt höchstens suchen
  int   ledBrightness = 64;           // Helligkeit der WS2812B (0..255)
  bool  received = false;             // true = Linux hat schon eine Konfiguration geschickt
};
Config cfg;                           // die aktuell gültige Konfiguration

// ------------------------------- Zustand: Motoren / A0 -------------------------------
bool motorsPresent = false;           // true = an A0 liegen 3,3 V (Motoren angeschlossen)
int  senseCounter = 0;                // zählt, wie oft A0 vom aktuellen Zustand abweicht
bool senseFirst = true;               // erste Messung wird direkt übernommen
unsigned long senseLastMs = 0;        // Zeitpunkt der letzten A0-Messung

// ------------------------------- Zustand: Kontaktschalter ----------------------------
bool switchPressed = false;           // entprellter Zustand des Kontaktschalters
int  switchCounter = 0;               // zählt abweichende Messungen zum Entprellen

// ------------------------------- Zustand: Schrittmotor -------------------------------
long  stepPos = 0;                    // aktuelle Position [Halbschritte], 0 = Schalter
long  stepTarget = 0;                 // Zielposition [Halbschritte]
float stepSpeed = 0.0f;               // aktuelle Geschwindigkeit [Halbschritte/s]
int   stepDir = 0;                    // aktuelle Fahrtrichtung (+1, -1 oder 0)
int   stepPhase = 0;                  // aktueller Index in der HALFSTEP-Tabelle
unsigned long stepLastUs = 0;         // Zeitpunkt des letzten Schrittes [µs]
unsigned long stepLastMoveMs = 0;     // Zeitpunkt der letzten Bewegung [ms]
bool  coilsOn = false;                // true = Spulen sind bestromt
bool  homed = false;                  // true = Referenzposition ist bekannt
int   homingState = HOMING_IDLE;      // aktueller Schritt der Referenzfahrt
long  homingTravel = 0;               // zurückgelegte Schritte bei der Referenzsuche

// ------------------------------- Zustand: Servos -------------------------------------
struct ServoAxis {                    // ein Servo mit sanfter Bewegung
  Servo servo;                        // Servo-Objekt der Bibliothek
  int   pin;                          // Signal-Pin
  float current;                      // aktuell ausgegebener Winkel [Grad]
  float target;                       // gewünschter Zielwinkel [Grad]
  bool  attached;                     // true = Servo bekommt Impulse
  unsigned long lastMoveMs;           // Zeitpunkt der letzten Winkeländerung
};
ServoAxis tiltAxis;                   // Servo 1: Kamera-/Plattform-Neigung
ServoAxis radarAxis;                  // Servo 2: Radar-Neigung
bool radarManual = false;             // true = Radar-Servo wird von Hand gestellt (Kalibrierung)
unsigned long servoLastMs = 0;        // Zeitpunkt der letzten Servo-Nachführung

// ------------------------------- Zustand: WS2812B ------------------------------------
uint32_t ledColor[2] = {0, 0};        // Grundfarbe je LED als 0xRRGGBB
uint16_t ledBlink[2] = {0, 0};        // Blinkzeit je LED (Dauer einer Phase in ms, 0 = Dauerlicht)
uint32_t flashColor = 0;              // Farbe des Blitz-Effekts (z.B. Lila bei Vibration)
int      flashCount = 0;              // Anzahl der Blitze (0 = kein Blitz aktiv)
unsigned long flashStartMs = 0;       // Startzeitpunkt des Blitz-Effekts
uint32_t ledSent[2] = {0xFFFFFFFF, 0xFFFFFFFF}; // zuletzt gesendete Farben (für Änderungsprüfung)
unsigned long ledLastMs = 0;          // Zeitpunkt der letzten LED-Berechnung
unsigned long ledLastSendMs = 0;      // Zeitpunkt der letzten Übertragung an die LEDs
const int WS_LEAD = 8;                // Null-Bytes vor den Daten (Leitung sicher auf LOW)
const int WS_TAIL = 4;                // Null-Bytes nach den Daten (Leitung endet auf LOW)
uint8_t wsBuffer[WS_LEAD + 2 * 24 + WS_TAIL]; // SPI-Puffer: 1 Byte pro WS2812-Bit

// ------------------------------- Zustand: LED-Matrix ---------------------------------
Arduino_LED_Matrix matrix;            // Objekt für die eingebaute LED-Matrix
const int MATRIX_W = 13;              // Breite der Matrix in Pixeln
const int MATRIX_H = 8;               // Höhe der Matrix in Pixeln
const int MAX_TEXT_COLS = 200;        // maximale Textlänge in Spalten
uint8_t matrixFrame[MATRIX_H][MATRIX_W]; // aktuelles Bild der Matrix (0/1 je Pixel)
uint8_t textCols[MAX_TEXT_COLS];      // gerenderter Text: je Spalte 8 Bit (Bit = Zeile)
int  textColCount = 0;                // Anzahl gültiger Spalten in textCols
bool textScrolling = false;           // true = Text ist zu breit und läuft durch
int  textOffset = 0;                  // Startspalte (stehend) bzw. Scroll-Position (laufend)
unsigned long scrollLastMs = 0;       // Zeitpunkt des letzten Scroll-Schrittes
bool matrixDirty = true;              // true = Matrix muss neu gezeichnet werden

// ------------------------------- Zustand: RD-03D Radar -------------------------------
const uint8_t RADAR_HEADER[4] = {0xAA, 0xFF, 0x03, 0x00}; // Kopf eines Datenrahmens
const int RADAR_FRAME_LEN = 30;       // Länge eines Rahmens: 4 Kopf + 3x8 Ziel + 2 Ende
uint8_t radarBuf[RADAR_FRAME_LEN];    // Empfangspuffer für einen Rahmen
int  radarIdx = 0;                    // Schreibposition im Empfangspuffer
int  radarVals[9] = {0};              // letzte Werte: x1,y1,v1,x2,y2,v2,x3,y3,v3
bool radarNew = false;                // true = neue Werte noch nicht gemeldet
bool radarOk = false;                 // true = Radar liefert Daten
bool radarCmdSent = false;            // true = Multi-Ziel-Befehl nach dem Start gesendet
unsigned long radarLastFrameMs = 0;   // Zeitpunkt des letzten gültigen Rahmens
unsigned long radarLastNotifyMs = 0;  // Zeitpunkt der letzten Meldung an Linux

// ------------------------------- Zustand: MPU6050 ------------------------------------
const uint8_t MPU_ADDR = 0x68;        // I2C-Adresse des MPU6050 (AD0 an GND)
bool  mpuOk = false;                  // true = MPU6050 antwortet
float accAvgX = 0, accAvgY = 0, accAvgZ = 0; // geglättete Beschleunigung je Achse [mg]
float vibBase = 0;                    // geglätteter Betrag der Beschleunigung [mg]
float vibPeak = 0;                    // größte Abweichung seit der letzten Statusmeldung [mg]
bool  imuFirst = true;                // erste Messung initialisiert die Mittelwerte
unsigned long imuLastMs = 0;          // Zeitpunkt der letzten Messung
unsigned long imuRetryMs = 0;         // Zeitpunkt für einen neuen Initialisierungsversuch
unsigned long vibLastEventMs = 0;     // Zeitpunkt der letzten Vibrationsmeldung

// ------------------------------- Zustand: Statusmeldung ------------------------------
unsigned long statusLastMs = 0;       // Zeitpunkt der letzten Statusmeldung

// =====================================================================================
//  Hilfsfunktionen
// =====================================================================================

float clampf(float v, float lo, float hi) {         // begrenzt v auf den Bereich lo..hi
  if (lo > hi) { float t = lo; lo = hi; hi = t; }   // falls Grenzen vertauscht sind: tauschen
  if (v < lo) return lo;                            // zu klein → untere Grenze
  if (v > hi) return hi;                            // zu groß → obere Grenze
  return v;                                         // sonst unverändert
}

void sendEvent(int code, int value) {               // meldet ein Ereignis an Linux
  Bridge.notify("mcu_event", code, value);          // Python-Funktion "mcu_event" aufrufen
}

// =====================================================================================
//  Schrittmotor
// =====================================================================================

void writeCoils(int phase) {                        // bestromt die Spulen für eine Phase
  uint8_t p = HALFSTEP[phase];                      // Bitmuster der Phase holen
  digitalWrite(PIN_STEP_IN1, (p >> 3) & 1);         // IN1 setzen
  digitalWrite(PIN_STEP_IN2, (p >> 2) & 1);         // IN2 setzen
  digitalWrite(PIN_STEP_IN3, (p >> 1) & 1);         // IN3 setzen
  digitalWrite(PIN_STEP_IN4, p & 1);                // IN4 setzen
  coilsOn = true;                                   // merken: Spulen sind an
}

void coilsOff() {                                   // schaltet alle Spulen stromlos (spart Strom)
  digitalWrite(PIN_STEP_IN1, LOW);                  // IN1 aus
  digitalWrite(PIN_STEP_IN2, LOW);                  // IN2 aus
  digitalWrite(PIN_STEP_IN3, LOW);                  // IN3 aus
  digitalWrite(PIN_STEP_IN4, LOW);                  // IN4 aus
  coilsOn = false;                                  // merken: Spulen sind aus
}

void doStep(int dir) {                              // führt genau einen Halbschritt aus
  int phys = cfg.stepInvert ? -dir : dir;           // logische in physikalische Richtung umrechnen
  stepPhase = (stepPhase + phys + 8) % 8;           // nächste Phase (mit Überlauf 0..7)
  writeCoils(stepPhase);                            // Spulen entsprechend bestromen
  stepPos += dir;                                   // Positionszähler mitführen
}

void stopStepper() {                                // hält den Motor an der aktuellen Stelle an
  stepTarget = stepPos;                             // Ziel = aktuelle Position
  stepSpeed = 0;                                    // Geschwindigkeit zurücksetzen
  stepDir = 0;                                      // keine Fahrtrichtung mehr
}

void updateStepper(unsigned long nowUs, unsigned long nowMs) {  // wird in jedem loop() aufgerufen
  if (!motorsPresent) {                             // ohne angeschlossene Motoren ...
    if (coilsOn) coilsOff();                        // ... Spulen sicher aus
    return;                                         // ... und nichts weiter tun
  }

  if (homingState == HOMING_SEARCH) {               // Referenzfahrt: Schalter suchen
    if (switchPressed) {                            // Schalter ist gedrückt → Referenz gefunden
      stepPos = 0;                                  // diese Stelle ist ab jetzt Position 0 (Offset)
      stopStepper();                                // kurz anhalten
      stepTarget = HOMING_BACKOFF;                  // vom Schalter wegfahren
      homingState = HOMING_LEAVE;                   // nächster Schritt der Referenzfahrt
      return;                                       // nächster Durchlauf fährt weiter
    }
    if (homingTravel > cfg.homingMaxSteps) {        // zu weit gefahren ohne Schalter
      stopStepper();                                // anhalten
      homingState = HOMING_FAILED;                  // Fehler merken
      sendEvent(EVT_HOMING_FAILED, (int)homingTravel); // Linux informieren
      Monitor.println("Referenzfahrt fehlgeschlagen: Schalter nicht gefunden"); // Debug-Ausgabe
      return;                                       // fertig
    }
    stepTarget = stepPos - 1;                       // immer einen Schritt weiter Richtung Schalter
  }

  long dist = stepTarget - stepPos;                 // verbleibende Strecke bis zum Ziel
  if (dist == 0) {                                  // Ziel erreicht
    stepSpeed = 0;                                  // Geschwindigkeit zurücksetzen
    stepDir = 0;                                    // keine Fahrtrichtung
    if (homingState == HOMING_LEAVE) {              // Referenzfahrt: Wegfahren beendet
      homingState = HOMING_DONE;                    // Referenzfahrt erfolgreich
      homed = true;                                 // Position ist jetzt bekannt
      sendEvent(EVT_HOMING_DONE, (int)stepPos);     // Linux informieren
      Monitor.println("Referenzfahrt erfolgreich"); // Debug-Ausgabe
    }
    if (coilsOn && (nowMs - stepLastMoveMs > COILS_OFF_MS)) coilsOff(); // im Stillstand Strom sparen
    return;                                         // nichts weiter zu tun
  }

  int dir = (dist > 0) ? 1 : -1;                    // gewünschte Fahrtrichtung
  if (dir < 0 && switchPressed && homingState != HOMING_SEARCH) { // Richtung Schalter, aber Schalter gedrückt
    stepPos = 0;                                    // wir stehen am Schalter → Position 0
    stopStepper();                                  // nicht weiter in den Schalter fahren
    homed = true;                                   // Position ist damit auch bekannt
    sendEvent(EVT_SWITCH_HIT, 0);                   // Linux informieren
    return;                                         // fertig
  }

  if (dir != stepDir) {                             // Richtungswechsel oder Start aus dem Stand
    stepDir = dir;                                  // neue Richtung merken
    stepSpeed = STEP_MIN_SPEED;                     // langsam anfahren
  }

  bool homingMove = (homingState == HOMING_SEARCH || homingState == HOMING_LEAVE); // Referenzfahrt aktiv?
  float vMax = homingMove ? HOMING_SPEED : cfg.stepMaxSpeed; // erlaubte Höchstgeschwindigkeit
  unsigned long interval = (unsigned long)(1000000.0f / stepSpeed); // Zeit zwischen zwei Schritten [µs]

  if (!coilsOn) {                                   // Spulen waren aus (Stromsparen)
    writeCoils(stepPhase);                          // zuerst aktuelle Phase bestromen (Rotor fixieren)
    stepLastUs = nowUs;                             // ab jetzt Zeit messen
    stepLastMoveMs = nowMs;                         // gilt als Bewegung
    return;                                         // erster Schritt kommt im nächsten Durchlauf
  }
  if (nowUs - stepLastUs < interval) return;        // noch nicht Zeit für den nächsten Schritt

  doStep(dir);                                      // einen Halbschritt fahren
  if (nowUs - stepLastUs > 2 * interval) stepLastUs = nowUs; // stark verspätet → Takt neu starten
  else stepLastUs += interval;                      // sonst exakt im Takt bleiben
  stepLastMoveMs = nowMs;                           // Zeitpunkt der Bewegung merken
  if (homingState == HOMING_SEARCH) homingTravel++; // Suchstrecke mitzählen

  stepSpeed += STEP_ACCEL * (interval / 1000000.0f); // beschleunigen (Rampe)
  if (homingState != HOMING_SEARCH) {               // beim Suchen gibt es kein festes Ziel
    float vBrake = sqrtf(2.0f * STEP_ACCEL * (float)labs(stepTarget - stepPos)); // Bremsgeschwindigkeit
    if (stepSpeed > vBrake) stepSpeed = vBrake;     // rechtzeitig vor dem Ziel abbremsen
  }
  stepSpeed = clampf(stepSpeed, STEP_MIN_SPEED, vMax); // auf erlaubten Bereich begrenzen
}

// =====================================================================================
//  Servos
// =====================================================================================

void attachServo(ServoAxis &ax) {                   // Servo einschalten (Impulse erzeugen)
  if (ax.attached) return;                          // schon aktiv → nichts tun
  ax.servo.attach(ax.pin, cfg.servoUsMin, cfg.servoUsMax); // Servo am Pin anmelden
  ax.attached = true;                               // merken: Servo aktiv
}

void detachServo(ServoAxis &ax) {                   // Servo ausschalten (keine Impulse mehr)
  if (!ax.attached) return;                         // schon aus → nichts tun
  ax.servo.detach();                                // Servo abmelden
  digitalWrite(ax.pin, LOW);                        // Signal sicher auf LOW legen
  ax.attached = false;                              // merken: Servo aus
}

void writeServo(ServoAxis &ax) {                    // gibt den aktuellen Winkel als Impuls aus
  if (!ax.attached) return;                         // ohne aktives Servo nichts ausgeben
  float us = cfg.servoUsMin + (cfg.servoUsMax - cfg.servoUsMin) * (ax.current / 180.0f); // Winkel → µs
  ax.servo.writeMicroseconds((int)(us + 0.5f));     // Impulslänge setzen (gerundet)
}

bool moveToward(ServoAxis &ax, float maxStep, unsigned long nowMs) { // bewegt den Winkel Richtung Ziel
  float diff = ax.target - ax.current;              // Abstand zum Ziel
  if (fabsf(diff) < 0.05f) {                        // praktisch am Ziel
    if (ax.current != ax.target) {                  // letzten Rest noch übernehmen
      ax.current = ax.target;                       // exakt auf Ziel setzen
      writeServo(ax);                               // ausgeben
    }
    return false;                                   // keine Bewegung mehr
  }
  if (diff > maxStep) diff = maxStep;               // Schrittweite nach oben begrenzen
  if (diff < -maxStep) diff = -maxStep;             // Schrittweite nach unten begrenzen
  ax.current += diff;                               // neuen Winkel berechnen
  ax.lastMoveMs = nowMs;                            // Zeitpunkt der Bewegung merken
  writeServo(ax);                                   // neuen Winkel ausgeben
  return true;                                      // es hat eine Bewegung gegeben
}

float radarCompTarget() {                           // berechnet den Radar-Winkel für "waagerecht"
  float tiltDelta = tiltAxis.current - cfg.tiltLevel; // wie weit ist die Plattform geneigt?
  float t = cfg.radarLevel + cfg.radarComp * tiltDelta; // Gegenbewegung des Radar-Servos
  return clampf(t, cfg.radarMin, cfg.radarMax);     // im erlaubten Bereich bleiben
}

void updateServos(unsigned long nowMs) {            // wird regelmäßig aufgerufen (50 Hz)
  if (nowMs - servoLastMs < SERVO_MS) return;       // noch nicht Zeit
  float dt = (nowMs - servoLastMs) / 1000.0f;       // vergangene Zeit in Sekunden
  servoLastMs = nowMs;                              // Zeitpunkt merken
  if (dt > 0.1f) dt = 0.1f;                         // nach Pausen keine großen Sprünge
  if (!motorsPresent || !cfg.received) return;      // ohne Motoren/Konfiguration nichts tun
  float maxStep = cfg.servoSpeed * dt;              // erlaubte Winkeländerung in diesem Takt
  moveToward(tiltAxis, maxStep, nowMs);             // Servo 1 sanft nachführen
  if (!radarManual && cfg.radarCompOn) {            // Radar automatisch waagerecht halten?
    radarAxis.target = radarCompTarget();           // Zielwinkel aus der Plattform-Neigung
  }
  moveToward(radarAxis, maxStep * 2.0f, nowMs);     // Servo 2 etwas schneller, damit es mithält
}

bool servosMoving(unsigned long nowMs) {            // bewegt sich gerade ein Motor?
  if (stepTarget != stepPos || homingState == HOMING_SEARCH) return true; // Schrittmotor fährt
  if (nowMs - stepLastMoveMs < QUIET_MS) return true; // Schrittmotor hat gerade erst angehalten
  if (nowMs - tiltAxis.lastMoveMs < QUIET_MS) return true;  // Servo 1 bewegt sich (noch)
  if (nowMs - radarAxis.lastMoveMs < QUIET_MS) return true; // Servo 2 bewegt sich (noch)
  return false;                                     // alles ruhig
}

// =====================================================================================
//  A0: Sind Servos und Schrittmotor angeschlossen?
// =====================================================================================

void applyMotorsPresent(bool present) {             // übernimmt einen neuen Zustand von A0
  motorsPresent = present;                          // neuen Zustand merken
  if (present) {                                    // Motoren sind (wieder) da
    if (cfg.received) {                             // nur mit gültiger Konfiguration einschalten
      attachServo(tiltAxis);                        // Servo 1 einschalten
      attachServo(radarAxis);                       // Servo 2 einschalten
      writeServo(tiltAxis);                         // aktuellen Winkel sofort ausgeben
      writeServo(radarAxis);                        // aktuellen Winkel sofort ausgeben
    }
    sendEvent(EVT_MOTORS_ON, 1);                    // Linux informieren
    Monitor.println("Motoren erkannt (A0 = 3,3 V)"); // Debug-Ausgabe
  } else {                                          // Motoren wurden entfernt
    detachServo(tiltAxis);                          // Servo 1 aus
    detachServo(radarAxis);                         // Servo 2 aus
    coilsOff();                                     // Schrittmotor aus
    stopStepper();                                  // Fahrauftrag verwerfen
    homed = false;                                  // Position ist nicht mehr sicher bekannt
    homingState = HOMING_IDLE;                      // laufende Referenzfahrt abbrechen
    sendEvent(EVT_MOTORS_OFF, 0);                   // Linux informieren
    Monitor.println("Keine Motoren (A0 = 0 V)");    // Debug-Ausgabe
  }
}

void updateMotorSense(unsigned long nowMs) {        // prüft A0 alle 50 ms
  if (nowMs - senseLastMs < SENSE_MS && !senseFirst) return; // noch nicht Zeit
  senseLastMs = nowMs;                              // Zeitpunkt merken
  float volts = analogRead(PIN_MOTOR_SENSE) * 3.3f / 4095.0f; // Messwert in Volt umrechnen
  bool reading = motorsPresent;                     // Standard: Zustand bleibt
  if (volts > 2.6f) reading = true;                 // deutlich über 2,6 V → angeschlossen
  if (volts < 2.0f) reading = false;                // deutlich unter 2,0 V → nicht angeschlossen
  if (senseFirst) {                                 // allererste Messung
    senseFirst = false;                             // ab jetzt normal entprellen
    if (reading) applyMotorsPresent(true);          // gleich übernehmen, falls Motoren da
    return;                                         // fertig
  }
  if (reading != motorsPresent) {                   // Messung weicht vom Zustand ab
    senseCounter++;                                 // Abweichung zählen
    if (senseCounter >= 4) {                        // 4x hintereinander (200 ms) → echt
      senseCounter = 0;                             // Zähler zurücksetzen
      applyMotorsPresent(reading);                  // neuen Zustand übernehmen
    }
  } else {                                          // Messung passt zum Zustand
    senseCounter = 0;                               // Zähler zurücksetzen
  }
}

// =====================================================================================
//  Kontaktschalter
// =====================================================================================

void updateSwitch() {                               // liest und entprellt den Schalter
  bool raw = (digitalRead(PIN_SWITCH) == LOW);      // LOW = gedrückt (Pull-up, Schalter gegen GND)
  if (raw != switchPressed) {                       // Abweichung vom entprellten Zustand
    switchCounter++;                                // Abweichung zählen
    if (switchCounter >= 3) {                       // 3x hintereinander gleich → übernehmen
      switchPressed = raw;                          // neuer entprellter Zustand
      switchCounter = 0;                            // Zähler zurücksetzen
    }
  } else {                                          // kein Unterschied
    switchCounter = 0;                              // Zähler zurücksetzen
  }
}

// =====================================================================================
//  WS2812B RGB-LEDs (über SPI)
// =====================================================================================
//  Trick: Bei 5 MHz SPI-Takt dauert ein SPI-Bit 200 ns. Jedes WS2812-Bit wird als ein
//  ganzes SPI-Byte gesendet:  "0" = 11000000 (400 ns HIGH),  "1" = 11110000 (800 ns HIGH).
//  Dadurch stimmt das Timing, ohne Interrupts zu sperren (wichtig für die Servos).

uint8_t scaleChannel(uint8_t v) {                   // Helligkeit anwenden
  return (uint8_t)(((uint16_t)v * (uint16_t)cfg.ledBrightness) / 255); // v * Helligkeit / 255
}

void wsSend(uint32_t c0, uint32_t c1) {             // sendet zwei Farben an die LED-Kette
  int i = 0;                                        // Schreibposition im Puffer
  for (int k = 0; k < WS_LEAD; k++) wsBuffer[i++] = 0x00; // Vorlauf: Leitung LOW
  uint32_t colors[2] = {c0, c1};                    // beide Farben in eine Liste
  for (int led = 0; led < 2; led++) {               // für jede LED ...
    uint8_t r = scaleChannel((colors[led] >> 16) & 0xFF); // Rotanteil (mit Helligkeit)
    uint8_t g = scaleChannel((colors[led] >> 8) & 0xFF);  // Grünanteil (mit Helligkeit)
    uint8_t b = scaleChannel(colors[led] & 0xFF);         // Blauanteil (mit Helligkeit)
    uint8_t grb[3] = {g, r, b};                     // WS2812B erwartet die Reihenfolge G, R, B
    for (int c = 0; c < 3; c++) {                   // für jeden Farbkanal ...
      for (int bit = 7; bit >= 0; bit--) {          // ... jedes Bit, höchstwertiges zuerst
        wsBuffer[i++] = (grb[c] & (1 << bit)) ? 0xF0 : 0xC0; // 1 → 11110000, 0 → 11000000
      }
    }
  }
  for (int k = 0; k < WS_TAIL; k++) wsBuffer[i++] = 0x00; // Nachlauf: Leitung LOW
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0)); // 5 MHz, Modus 0
  SPI.transfer(wsBuffer, i);                        // ganzen Puffer in einem Rutsch senden
  SPI.endTransaction();                             // SPI wieder freigeben
}

void updateLeds(unsigned long nowMs) {              // berechnet die LED-Farben (Blinken, Blitzen)
  if (nowMs - ledLastMs < LED_MS) return;           // noch nicht Zeit
  ledLastMs = nowMs;                                // Zeitpunkt merken
  uint32_t out[2];                                  // die jetzt anzuzeigenden Farben
  bool flashing = false;                            // läuft gerade ein Blitz-Effekt?
  if (flashCount > 0) {                             // Blitz-Effekt angefordert
    unsigned long phase = (nowMs - flashStartMs) / FLASH_MS; // welche Hell/Dunkel-Phase?
    if (phase < (unsigned long)(flashCount * 2)) {  // Effekt noch nicht vorbei
      flashing = true;                              // Blitz hat Vorrang
      uint32_t c = (phase % 2 == 0) ? flashColor : 0; // gerade Phase = hell, ungerade = dunkel
      out[0] = c;                                   // beide LEDs ...
      out[1] = c;                                   // ... blitzen gemeinsam
    } else {                                        // Effekt ist vorbei
      flashCount = 0;                               // Blitz beenden
    }
  }
  if (!flashing) {                                  // normaler Betrieb
    for (int i = 0; i < 2; i++) {                   // für jede LED ...
      bool on = (ledBlink[i] == 0) || ((nowMs / ledBlink[i]) % 2 == 0); // an oder (blinkend) aus?
      out[i] = on ? ledColor[i] : 0;                // Farbe oder schwarz
    }
  }
  bool changed = (out[0] != ledSent[0]) || (out[1] != ledSent[1]); // hat sich etwas geändert?
  if (changed || (nowMs - ledLastSendMs > LED_REFRESH_MS)) { // bei Änderung oder zur Auffrischung
    wsSend(out[0], out[1]);                         // an die LEDs senden
    ledSent[0] = out[0];                            // gesendete Farbe merken
    ledSent[1] = out[1];                            // gesendete Farbe merken
    ledLastSendMs = nowMs;                          // Zeitpunkt merken
  }
}

// =====================================================================================
//  LED-Matrix (13 x 8)
// =====================================================================================

void setMatrixText(const String &s) {               // bereitet einen neuen Text vor
  textColCount = 0;                                 // Spaltenliste leeren
  for (unsigned int n = 0; n < s.length(); n++) {   // jedes Zeichen des Textes
    uint8_t ch = (uint8_t)s.charAt(n);              // Zeichencode holen
    if (ch < 32 || ch > 126) ch = '?';              // nur druckbare ASCII-Zeichen
    const uint8_t *glyph = Font_4x6.data[ch];       // Pixelmuster des Zeichens (6 Zeilen)
    for (int x = 0; x < Font_4x6.width; x++) {      // jede Spalte des Zeichens (4 breit)
      uint8_t col = 0;                              // Bits dieser Spalte
      for (int y = 0; y < Font_4x6.height && glyph != NULL; y++) { // jede Zeile des Zeichens
        if (glyph[y] & (0x80 >> x)) col |= (1 << (y + 1)); // Pixel gesetzt → Bit (1 Zeile Abstand oben)
      }
      if (textColCount < MAX_TEXT_COLS) textCols[textColCount++] = col; // Spalte speichern
    }
  }
  int visible = textColCount > 0 ? textColCount - 1 : 0; // letzte Spalte ist nur Zeichenabstand
  textScrolling = visible > MATRIX_W;               // passt der Text nicht → Lauftext
  if (textScrolling) {                              // Lauftext vorbereiten
    for (int k = 0; k < 4 && textColCount < MAX_TEXT_COLS; k++) textCols[textColCount++] = 0; // Lücke am Ende
    textOffset = 0;                                 // von vorne beginnen
  } else {                                          // stehender Text
    textOffset = (MATRIX_W - visible) / 2;          // waagerecht zentrieren
  }
  matrixDirty = true;                               // Matrix neu zeichnen
}

void drawMatrix() {                                 // überträgt den Text auf die Matrix
  for (int x = 0; x < MATRIX_W; x++) {              // jede Spalte der Matrix
    uint8_t col = 0;                                // Bits dieser Spalte
    if (textColCount > 0) {                         // gibt es überhaupt Text?
      if (textScrolling) {                          // Lauftext
        col = textCols[(textOffset + x) % textColCount]; // Spalte mit Umlauf holen
      } else {                                      // stehender Text
        int src = x - textOffset;                   // Spalte im Text
        if (src >= 0 && src < textColCount) col = textCols[src]; // nur gültige Spalten
      }
    }
    for (int y = 0; y < MATRIX_H; y++) {            // jede Zeile
      matrixFrame[y][x] = (col >> y) & 1;           // Pixel an/aus
    }
  }
  matrix.renderBitmap(matrixFrame, MATRIX_H, MATRIX_W); // Bild an die Matrix übergeben
}

void updateMatrix(unsigned long nowMs) {            // Lauftext weiterschieben und zeichnen
  if (textScrolling && (nowMs - scrollLastMs >= SCROLL_MS)) { // Zeit für den nächsten Schritt?
    scrollLastMs = nowMs;                           // Zeitpunkt merken
    textOffset = (textOffset + 1) % textColCount;   // eine Spalte weiter
    matrixDirty = true;                             // neu zeichnen
  }
  if (matrixDirty) {                                // nur bei Änderungen zeichnen
    drawMatrix();                                   // zeichnen
    matrixDirty = false;                            // erledigt
  }
}

// =====================================================================================
//  RD-03D Radar
// =====================================================================================

void radarSendMultiTarget() {                       // schaltet das Radar auf "mehrere Ziele"
  const uint8_t cmd[12] = {0xFD, 0xFC, 0xFB, 0xFA,  // Befehlskopf
                           0x02, 0x00,              // Länge der Nutzdaten (2 Byte)
                           0x90, 0x00,              // Befehl 0x0090 = Multi-Target-Modus
                           0x04, 0x03, 0x02, 0x01}; // Befehlsende
  Serial1.write(cmd, sizeof(cmd));                  // Befehl senden
  radarCmdSent = true;                              // merken: gesendet
}

int radarDecode(uint8_t lo, uint8_t hi) {           // wandelt einen Radar-Wert um
  int v = ((hi & 0x7F) << 8) | lo;                  // Betrag aus 15 Bit
  return (hi & 0x80) ? v : -v;                      // oberstes Bit 1 = positiv, 0 = negativ
}

void radarParseFrame(unsigned long nowMs) {         // wertet einen vollständigen Rahmen aus
  for (int t = 0; t < 3; t++) {                     // 3 mögliche Ziele
    int o = 4 + t * 8;                              // Startposition des Ziels im Rahmen
    radarVals[t * 3 + 0] = radarDecode(radarBuf[o + 0], radarBuf[o + 1]); // X [mm]
    radarVals[t * 3 + 1] = radarDecode(radarBuf[o + 2], radarBuf[o + 3]); // Y [mm]
    radarVals[t * 3 + 2] = radarDecode(radarBuf[o + 4], radarBuf[o + 5]); // Geschwindigkeit [cm/s]
  }
  if (!radarOk) {                                   // Radar meldet sich (wieder)
    radarOk = true;                                 // Radar ist da
    radarSendMultiTarget();                         // sicherheitshalber Multi-Ziel-Modus setzen
  }
  radarLastFrameMs = nowMs;                         // Zeitpunkt merken
  radarNew = true;                                  // neue Werte vorhanden
}

void updateRadar(unsigned long nowMs) {             // liest Bytes vom Radar
  int budget = 128;                                 // höchstens 128 Bytes pro Durchlauf
  while (Serial1.available() > 0 && budget-- > 0) { // solange Daten da sind
    uint8_t b = (uint8_t)Serial1.read();            // ein Byte lesen
    if (radarIdx < 4) {                             // wir suchen noch den Kopf
      if (b == RADAR_HEADER[radarIdx]) {            // passendes Kopfbyte
        radarBuf[radarIdx++] = b;                   // speichern und weiter
      } else if (b == RADAR_HEADER[0]) {            // könnte ein neuer Kopf sein
        radarBuf[0] = b;                            // als erstes Kopfbyte nehmen
        radarIdx = 1;                               // weiter beim zweiten Byte
      } else {                                      // passt nicht
        radarIdx = 0;                               // von vorne suchen
      }
      continue;                                     // nächstes Byte
    }
    radarBuf[radarIdx++] = b;                       // Datenbyte speichern
    if (radarIdx == RADAR_FRAME_LEN) {              // Rahmen vollständig
      if (radarBuf[28] == 0x55 && radarBuf[29] == 0xCC) radarParseFrame(nowMs); // Ende prüfen → auswerten
      radarIdx = 0;                                 // nächsten Rahmen suchen
    }
  }
  if (!radarCmdSent && nowMs > 1500) radarSendMultiTarget(); // einmal nach dem Start Modus setzen
  if (radarOk && (nowMs - radarLastFrameMs > RADAR_TIMEOUT_MS)) { // lange nichts mehr gehört
    radarOk = false;                                // Radar gilt als nicht verfügbar
    for (int i = 0; i < 9; i++) radarVals[i] = 0;   // alte Ziele löschen
    radarNew = true;                                // leere Liste melden
  }
  if (radarNew && (nowMs - radarLastNotifyMs >= RADAR_NOTIFY_MS)) { // Meldung fällig?
    radarLastNotifyMs = nowMs;                      // Zeitpunkt merken
    radarNew = false;                               // gemeldet
    Bridge.notify("mcu_radar",                      // Python-Funktion "mcu_radar" aufrufen
                  radarVals[0], radarVals[1], radarVals[2],  // Ziel 1: x, y, v
                  radarVals[3], radarVals[4], radarVals[5],  // Ziel 2: x, y, v
                  radarVals[6], radarVals[7], radarVals[8]); // Ziel 3: x, y, v
  }
}

// =====================================================================================
//  MPU6050 (Beschleunigung → Vibration und Lage)
// =====================================================================================

bool mpuWrite(uint8_t reg, uint8_t val) {           // schreibt ein Register des MPU6050
  Wire.beginTransmission(MPU_ADDR);                 // Übertragung an den MPU6050 beginnen
  Wire.write(reg);                                  // Registeradresse
  Wire.write(val);                                  // Wert
  return Wire.endTransmission() == 0;               // 0 = erfolgreich
}

bool mpuInit() {                                    // richtet den MPU6050 ein
  if (!mpuWrite(0x6B, 0x80)) return false;          // PWR_MGMT_1: Reset (keine Antwort → kein Sensor)
  delay(100);                                       // Reset abwarten (nur beim Start)
  mpuWrite(0x6B, 0x01);                             // PWR_MGMT_1: aufwecken, Takt vom Gyro X
  mpuWrite(0x19, 4);                                // SMPLRT_DIV: 1 kHz / (1+4) = 200 Hz
  mpuWrite(0x1A, 0x02);                             // CONFIG: Tiefpass ca. 94 Hz (gut für Vibration)
  mpuWrite(0x1C, 0x00);                             // ACCEL_CONFIG: Messbereich ±2 g
  return mpuWrite(0x1B, 0x00);                      // GYRO_CONFIG: ±250 °/s (wird nicht genutzt)
}

bool mpuRead(float &ax, float &ay, float &az) {     // liest die Beschleunigung in mg
  Wire.beginTransmission(MPU_ADDR);                 // Übertragung beginnen
  Wire.write(0x3B);                                 // Startregister ACCEL_XOUT_H
  if (Wire.endTransmission() != 0) return false;    // keine Antwort → Fehler
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6) != 6) return false; // 6 Bytes anfordern
  int16_t rx = (int16_t)((Wire.read() << 8) | Wire.read()); // X (High-, dann Low-Byte)
  int16_t ry = (int16_t)((Wire.read() << 8) | Wire.read()); // Y
  int16_t rz = (int16_t)((Wire.read() << 8) | Wire.read()); // Z
  ax = rx * 1000.0f / 16384.0f;                     // in mg umrechnen (16384 LSB = 1 g)
  ay = ry * 1000.0f / 16384.0f;                     // in mg umrechnen
  az = rz * 1000.0f / 16384.0f;                     // in mg umrechnen
  return true;                                      // erfolgreich
}

void updateImu(unsigned long nowMs) {               // liest den MPU6050 und erkennt Vibrationen
  if (!mpuOk) {                                     // Sensor (noch) nicht bereit
    if (nowMs >= imuRetryMs) {                      // Zeit für einen neuen Versuch?
      mpuOk = mpuInit();                            // initialisieren versuchen
      imuFirst = true;                              // Mittelwerte neu beginnen
      imuRetryMs = nowMs + 3000;                    // nächster Versuch frühestens in 3 s
    }
    return;                                         // ohne Sensor nichts weiter tun
  }
  if (nowMs - imuLastMs < IMU_MS) return;           // noch nicht Zeit
  imuLastMs = nowMs;                                // Zeitpunkt merken
  float ax, ay, az;                                 // Messwerte
  if (!mpuRead(ax, ay, az)) {                       // Lesen fehlgeschlagen
    mpuOk = false;                                  // Sensor als fehlend markieren
    imuRetryMs = nowMs + 1000;                      // in 1 s neu versuchen
    return;                                         // fertig
  }
  float mag = sqrtf(ax * ax + ay * ay + az * az);   // Betrag der Beschleunigung
  if (imuFirst) {                                   // erste Messung
    imuFirst = false;                               // ab jetzt normal
    accAvgX = ax; accAvgY = ay; accAvgZ = az;       // Mittelwerte starten mit Messwert
    vibBase = mag;                                  // Grundwert starten mit Messwert
    return;                                         // fertig
  }
  accAvgX += 0.02f * (ax - accAvgX);                // gleitender Mittelwert X (Lage)
  accAvgY += 0.02f * (ay - accAvgY);                // gleitender Mittelwert Y (Lage)
  accAvgZ += 0.02f * (az - accAvgZ);                // gleitender Mittelwert Z (Lage)
  vibBase += 0.01f * (mag - vibBase);               // langsamer Grundwert (ca. 1 g in Ruhe)
  float dev = fabsf(mag - vibBase);                 // Abweichung = Erschütterung
  if (dev > vibPeak) vibPeak = dev;                 // größten Wert für die Statusmeldung merken
  if (dev > cfg.vibThreshold                        // Schwellwert überschritten ...
      && !servosMoving(nowMs)                       // ... und die eigenen Motoren sind ruhig ...
      && (nowMs - vibLastEventMs > VIB_GAP_MS)) {   // ... und letzte Meldung ist lange her
    vibLastEventMs = nowMs;                         // Zeitpunkt merken
    flashColor = 0x8000FF;                          // Lila
    flashCount = 3;                                 // dreimal blinken
    flashStartMs = nowMs;                           // ab jetzt
    Bridge.notify("mcu_vibration", (int)dev);       // Linux informieren (für Aufwachen)
  }
}

// =====================================================================================
//  Statusmeldung an Linux
// =====================================================================================

void sendStatus(unsigned long nowMs) {              // meldet regelmäßig den Zustand
  if (nowMs - statusLastMs < STATUS_MS) return;     // noch nicht Zeit
  statusLastMs = nowMs;                             // Zeitpunkt merken
  Bridge.notify("mcu_status",                       // Python-Funktion "mcu_status" aufrufen
                (int)motorsPresent,                 // 0: Motoren angeschlossen?
                (int)cfg.received,                  // 1: Konfiguration erhalten?
                (int)homed,                         // 2: Referenzposition bekannt?
                homingState,                        // 3: Zustand der Referenzfahrt
                (int)stepPos,                       // 4: Position Drehachse [Halbschritte]
                (int)(stepTarget != stepPos),       // 5: Drehachse fährt?
                (int)(tiltAxis.current * 10.0f),    // 6: Winkel Servo 1 [0,1 Grad]
                (int)(radarAxis.current * 10.0f),   // 7: Winkel Servo 2 [0,1 Grad]
                (int)switchPressed,                 // 8: Kontaktschalter gedrückt?
                (int)accAvgX,                       // 9: Beschleunigung X [mg]
                (int)accAvgY,                       // 10: Beschleunigung Y [mg]
                (int)accAvgZ,                       // 11: Beschleunigung Z [mg]
                (int)vibPeak,                       // 12: stärkste Erschütterung [mg]
                (int)mpuOk,                         // 13: MPU6050 vorhanden?
                (int)radarOk);                      // 14: Radar liefert Daten?
  vibPeak = 0;                                      // Spitzenwert für die nächste Periode zurücksetzen
}

// =====================================================================================
//  RPC-Funktionen (werden von Python aus aufgerufen)
// =====================================================================================

bool rpcSetConfig(std::vector<int> v) {             // übernimmt die Konfiguration als Zahlenliste
  if (v.size() < 20) return false;                  // zu kurz → ablehnen
  cfg.tiltMin      = v[0] / 10.0f;                  // 0: Servo 1 Minimum [0,1 Grad]
  cfg.tiltMax      = v[1] / 10.0f;                  // 1: Servo 1 Maximum
  cfg.tiltLevel    = v[2] / 10.0f;                  // 2: Servo 1 waagerecht
  cfg.radarMin     = v[3] / 10.0f;                  // 3: Servo 2 Minimum
  cfg.radarMax     = v[4] / 10.0f;                  // 4: Servo 2 Maximum
  cfg.radarLevel   = v[5] / 10.0f;                  // 5: Servo 2 waagerecht
  cfg.radarComp    = v[6] / 100.0f;                 // 6: Ausgleichsfaktor [x100]
  cfg.radarCompOn  = v[7] != 0;                     // 7: Ausgleich ein/aus
  cfg.panMin       = v[8];                          // 8: Drehachse Minimum [Halbschritte]
  cfg.panMax       = v[9];                          // 9: Drehachse Maximum [Halbschritte]
  cfg.stepInvert   = v[10] != 0;                    // 10: Drehrichtung umkehren
  cfg.stepMaxSpeed = clampf(v[11], STEP_MIN_SPEED, 1000.0f); // 11: Höchstgeschwindigkeit
  cfg.servoSpeed   = clampf(v[12], 10.0f, 360.0f);  // 12: Servo-Geschwindigkeit [Grad/s]
  cfg.vibThreshold = v[13];                         // 13: Vibrationsschwelle [mg]
  int usMin = v[14];                                // 14: Impulslänge 0 Grad [µs]
  int usMax = v[15];                                // 15: Impulslänge 180 Grad [µs]
  cfg.calibrated   = v[16] != 0;                    // 16: Kalibrierung 1 abgeschlossen
  cfg.homingMaxSteps = v[17];                       // 17: max. Suchweg der Referenzfahrt
  cfg.ledBrightness = (int)clampf(v[18], 0, 255);   // 18: LED-Helligkeit
  // v[19] ist reserviert (Versionsnummer der Liste)
  bool usChanged = (usMin != cfg.servoUsMin) || (usMax != cfg.servoUsMax); // Impulsgrenzen geändert?
  cfg.servoUsMin = usMin;                           // übernehmen
  cfg.servoUsMax = usMax;                           // übernehmen
  bool first = !cfg.received;                       // erste Konfiguration seit dem Start?
  cfg.received = true;                              // ab jetzt gültig
  if (first) {                                      // beim ersten Mal ...
    tiltAxis.current = tiltAxis.target = cfg.tiltLevel;   // ... Servo 1 waagerecht
    radarAxis.current = radarAxis.target = cfg.radarLevel; // ... Servo 2 waagerecht
  }
  if (motorsPresent) {                              // Motoren angeschlossen?
    if (usChanged) {                                // Impulsgrenzen geändert → neu anmelden
      detachServo(tiltAxis);                        // Servo 1 abmelden
      detachServo(radarAxis);                       // Servo 2 abmelden
    }
    attachServo(tiltAxis);                          // Servo 1 (wieder) einschalten
    attachServo(radarAxis);                         // Servo 2 (wieder) einschalten
    writeServo(tiltAxis);                           // Winkel sofort ausgeben
    writeServo(radarAxis);                          // Winkel sofort ausgeben
  }
  sendEvent(EVT_CONFIG_OK, FW_VERSION);             // Linux bestätigen
  return true;                                      // erfolgreich
}

bool rpcTilt(int deg10, int raw) {                  // stellt Servo 1 (Kamera-Neigung)
  if (!motorsPresent || !cfg.received) return false; // ohne Motoren/Konfiguration nicht möglich
  float d = deg10 / 10.0f;                          // in Grad umrechnen
  if (raw || !cfg.calibrated) d = clampf(d, 0.0f, 180.0f); // Kalibriermodus: voller Bereich
  else d = clampf(d, cfg.tiltMin, cfg.tiltMax);     // Normalbetrieb: nur kalibrierter Bereich
  tiltAxis.target = d;                              // neues Ziel (wird sanft angefahren)
  return true;                                      // erfolgreich
}

bool rpcRadar(int deg10) {                          // stellt Servo 2 (Radar) von Hand
  if (!motorsPresent || !cfg.received) return false; // ohne Motoren/Konfiguration nicht möglich
  if (deg10 < 0) {                                  // negativer Wert = zurück zur Automatik
    radarManual = false;                            // Automatik (waagerecht halten) an
    return true;                                    // erfolgreich
  }
  radarManual = true;                               // Handbetrieb
  radarAxis.target = clampf(deg10 / 10.0f, 0.0f, 180.0f); // Zielwinkel setzen
  return true;                                      // erfolgreich
}

bool rpcPanTo(int pos) {                            // fährt die Drehachse auf eine Position
  if (!motorsPresent || !homed) return false;       // nur mit Motoren und bekannter Position
  if (homingState == HOMING_SEARCH || homingState == HOMING_LEAVE) return false; // nicht während Referenzfahrt
  long p = pos;                                     // Zielposition
  if (cfg.calibrated) p = constrain(p, cfg.panMin, cfg.panMax); // im kalibrierten Bereich bleiben
  else p = constrain(p, 0L, cfg.homingMaxSteps);    // sonst wenigstens grob begrenzen
  stepTarget = p;                                   // neues Ziel
  return true;                                      // erfolgreich
}

bool rpcPanJog(int delta) {                         // fährt die Drehachse relativ (Kalibrierung)
  if (!motorsPresent) return false;                 // ohne Motoren nicht möglich
  if (homingState == HOMING_SEARCH || homingState == HOMING_LEAVE) return false; // nicht während Referenzfahrt
  long p = stepTarget + delta;                      // neues Ziel relativ zum alten Ziel
  if (homed) p = constrain(p, 0L, cfg.homingMaxSteps); // mit Referenz: nicht hinter den Schalter
  stepTarget = p;                                   // neues Ziel (Schalter stoppt Richtung 0)
  return true;                                      // erfolgreich
}

bool rpcHome() {                                    // startet die Referenzfahrt
  if (!motorsPresent) return false;                 // ohne Motoren nicht möglich
  homed = false;                                    // Position gilt als unbekannt
  homingTravel = 0;                                 // Suchstrecke zurücksetzen
  stopStepper();                                    // laufende Fahrt beenden
  homingState = HOMING_SEARCH;                      // Suche nach dem Schalter beginnt
  Monitor.println("Referenzfahrt gestartet");       // Debug-Ausgabe
  return true;                                      // erfolgreich
}

bool rpcStop() {                                    // hält den Schrittmotor sofort an
  if (homingState == HOMING_SEARCH || homingState == HOMING_LEAVE) homingState = HOMING_IDLE; // Referenzfahrt abbrechen
  stopStepper();                                    // anhalten
  return true;                                      // erfolgreich
}

bool rpcLed(int idx, int rgb, int blinkMs) {        // setzt Farbe und Blinken einer LED
  if (blinkMs < 0) blinkMs = 0;                     // negative Zeit → Dauerlicht
  if (idx == 0 || idx == 2) {                       // LED 1 (oder beide)
    ledColor[0] = (uint32_t)rgb & 0xFFFFFF;         // Farbe übernehmen
    ledBlink[0] = (uint16_t)blinkMs;                // Blinkzeit übernehmen
  }
  if (idx == 1 || idx == 2) {                       // LED 2 (oder beide)
    ledColor[1] = (uint32_t)rgb & 0xFFFFFF;         // Farbe übernehmen
    ledBlink[1] = (uint16_t)blinkMs;                // Blinkzeit übernehmen
  }
  return true;                                      // erfolgreich
}

bool rpcFlash(int rgb, int count) {                 // lässt beide LEDs mehrfach aufblitzen
  flashColor = (uint32_t)rgb & 0xFFFFFF;            // Blitzfarbe
  flashCount = count;                               // Anzahl der Blitze
  flashStartMs = millis();                          // ab jetzt
  return true;                                      // erfolgreich
}

bool rpcText(String s) {                            // zeigt einen Text auf der LED-Matrix
  setMatrixText(s);                                 // Text vorbereiten (leer = Matrix aus)
  return true;                                      // erfolgreich
}

int rpcVersion() {                                  // liefert die Sketch-Version
  return FW_VERSION;                                // Versionsnummer
}

// =====================================================================================
//  setup() und loop()
// =====================================================================================

void setup() {
  pinMode(PIN_SWITCH, INPUT_PULLUP);                // Schalter mit internem Pull-up
  pinMode(PIN_MPU_INT, INPUT);                      // MPU-Interrupt (derzeit ungenutzt)
  pinMode(PIN_STEP_IN1, OUTPUT);                    // Schrittmotor-Ausgänge ...
  pinMode(PIN_STEP_IN2, OUTPUT);                    // ...
  pinMode(PIN_STEP_IN3, OUTPUT);                    // ...
  pinMode(PIN_STEP_IN4, OUTPUT);                    // ... als Ausgänge
  coilsOff();                                       // Spulen zu Beginn stromlos
  pinMode(PIN_SERVO_TILT, OUTPUT);                  // Servo-Pins als Ausgang ...
  pinMode(PIN_SERVO_RADAR, OUTPUT);                 // ...
  digitalWrite(PIN_SERVO_TILT, LOW);                // ... und auf LOW (kein Impuls)
  digitalWrite(PIN_SERVO_RADAR, LOW);               // ...
  tiltAxis.pin = PIN_SERVO_TILT;                    // Pin von Servo 1 merken
  radarAxis.pin = PIN_SERVO_RADAR;                  // Pin von Servo 2 merken
  tiltAxis.current = tiltAxis.target = 90.0f;       // Startwinkel (wird von Konfig überschrieben)
  radarAxis.current = radarAxis.target = 90.0f;     // Startwinkel (wird von Konfig überschrieben)
  tiltAxis.attached = false;                        // noch keine Impulse
  radarAxis.attached = false;                       // noch keine Impulse
  tiltAxis.lastMoveMs = 0;                          // noch keine Bewegung
  radarAxis.lastMoveMs = 0;                         // noch keine Bewegung
  analogReadResolution(12);                         // A0 mit 12 Bit (0..4095) messen

  SPI.begin();                                      // SPI für die WS2812B starten
  wsSend(0, 0);                                     // LEDs zu Beginn aus

  matrix.begin();                                   // LED-Matrix starten
  setMatrixText("");                                // Matrix leer

  Wire.begin();                                     // I2C starten
  Wire.setClock(400000);                            // 400 kHz (Fast Mode)

  Serial1.begin(256000);                            // UART zum RD-03D (256000 Baud)

  Bridge.begin();                                   // Bridge zur Linux-Seite starten
  Monitor.begin();                                  // Debug-Ausgabe über die Bridge
  Bridge.provide_safe("set_config", rpcSetConfig);  // Funktionen für Python freigeben ...
  Bridge.provide_safe("tilt", rpcTilt);             // ... "safe" = läuft im loop()-Thread
  Bridge.provide_safe("radar_servo", rpcRadar);     // ...
  Bridge.provide_safe("pan_to", rpcPanTo);          // ...
  Bridge.provide_safe("pan_jog", rpcPanJog);        // ...
  Bridge.provide_safe("home", rpcHome);             // ...
  Bridge.provide_safe("stop", rpcStop);             // ...
  Bridge.provide_safe("led", rpcLed);               // ...
  Bridge.provide_safe("flash", rpcFlash);           // ...
  Bridge.provide_safe("text", rpcText);             // ...
  Bridge.provide_safe("version", rpcVersion);       // ...

  rpcLed(2, 0xFF0000, 500);                         // bis Linux sich meldet: beide LEDs rot blinkend
  Monitor.println("Watchmen MCU gestartet");        // Debug-Ausgabe
}

void loop() {
  unsigned long nowMs = millis();                   // aktuelle Zeit in Millisekunden
  unsigned long nowUs = micros();                   // aktuelle Zeit in Mikrosekunden
  updateMotorSense(nowMs);                          // A0: Motoren angeschlossen?
  updateSwitch();                                   // Kontaktschalter entprellen
  updateStepper(nowUs, nowMs);                      // Schrittmotor (und Referenzfahrt)
  updateServos(nowMs);                              // Servos sanft nachführen
  updateRadar(nowMs);                               // Radar-Daten lesen und melden
  updateImu(nowMs);                                 // MPU6050 lesen, Vibration erkennen
  updateLeds(nowMs);                                // WS2812B aktualisieren
  updateMatrix(nowMs);                              // LED-Matrix aktualisieren
  sendStatus(nowMs);                                // Zustand an Linux melden
}                                                   // danach bearbeitet die Bridge eingehende Aufrufe

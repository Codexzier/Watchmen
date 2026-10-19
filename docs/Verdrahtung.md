# Verdrahtung und Pinbelegung

Alle Angaben beziehen sich auf den **Arduino UNO Q** (MCU STM32U585) im UNO-Formfaktor. Die
Shield-Platine in `hardware/` setzt genau diese Belegung um. Ohne Shield (Steckbrett) gilt dieselbe
Tabelle.

## Pinbelegung UNO Q

| Pin | Richtung | Funktion | Bemerkung |
|---|---|---|---|
| D0 (RX, `Serial1`) | Eingang | RD-03D **TX** | über 470 Ω, 256000 Baud, 3,3-V-Pegel |
| D1 (TX, `Serial1`) | Ausgang | RD-03D **RX** | über 470 Ω |
| D2 | Eingang | Kontaktschalter (Referenz der Drehachse) | schaltet gegen GND, interner Pull-up, RC-Filter 1 kΩ / 100 nF |
| D3 | Eingang | MPU6050 INT | reserviert (Firmware fragt den Sensor zyklisch ab) |
| D4 | Ausgang | ULN2003 IN1 → Spule Orange | |
| D7 | Ausgang | ULN2003 IN2 → Spule Gelb | |
| D8 | Ausgang | ULN2003 IN3 → Spule Rosa | |
| D9 | Ausgang | ULN2003 IN4 → Spule Blau | |
| D10, D12, D13 | – | **frei lassen** | vom SPI belegt (NSS, MISO, SCK) |
| D11 (SPI MOSI) | Ausgang | WS2812B DIN | über Pegelwandler 74AHCT1G125 (3,3 V → 5 V) und 330 Ω |
| D5, D6 | – | frei | |
| SDA / SCL (D20/D21) | I²C | MPU6050 SDA / SCL | `Wire`, 400 kHz |
| A0 | Eingang (analog) | Motor-Erkennung | **3,3 V = Servos/Schrittmotor angeschlossen** |
| A1 | Ausgang | Servo 1 (Neigung Kamera/Plattform) | über 220 Ω |
| A2 | Ausgang | Servo 2 (Neigung Radar, Lageausgleich) | über 220 Ω |
| A3–A5 | – | frei | |

> Die Servos hängen an **A1/A2** (nicht an D5/D6), weil deren Stecker auf dem Shield direkt neben der
> Analog-Leiste liegen. Die Servo-Bibliothek erzeugt die Impulse per Software und funktioniert an
> jedem Digital-/Analog-Pin.

## Anschlüsse auf dem Shield

| Stecker | Belegung (Pin 1 → n) | Gegenstück |
|---|---|---|
| **J5 MOTOR 5V** (Schraubklemme) | `+` 5–6 V, `GND` | Netzteil für Servos und Schrittmotor, mind. 3 A |
| **J6 STEPPER** (JST-XH 5) | BL, PK, YE, OR, RD | Stecker des 28BYJ-48 direkt aufstecken |
| **J7 SERVO1** | S, +, − | Servo Kamera-Neigung (orange, rot, braun) |
| **J8 SERVO2** | S, +, − | Servo Radar-Neigung |
| **J9 REF** | SW, GND | Kontaktschalter (Schließer) |
| **J10 RD-03D** | 5V, GND, TX, RX | AI-Thinker RD-03D (TX des Radars → D0) |
| **J11 MPU6050** (Buchse) | VCC, GND, SCL, SDA, XDA, XCL, AD0, INT | GY-521-Modul (VCC = 3,3 V, AD0 an GND → Adresse 0x68) |
| **J12 WS2812B** | 5V, DIN, GND | LED 1 → DOUT → LED 2 (Kette) |
| **JP1 A0-QUELLE** (Steckbrücke) | DIV, A0, 3V3 | DIV–A0: A0 aus der Motor-Versorgung · A0–3V3: Motoren immer „da“ · offen: keine Motoren |
| **JP2 5V>VMOT** (Lötbrücke) | | verbindet die 5 V des UNO Q mit der Motor-Versorgung – **nur** für sehr kleine Lasten/Tests |

### Stromversorgung

* **Logik**: UNO Q über USB-C (5 V). Davon gespeist: RD-03D, WS2812B, Pegelwandler (5 V) sowie MPU6050 (3,3 V).
* **Motoren** (VMOT): eigenes 5-V-Netzteil an J5. Verpolschutz (P-MOSFET AO3401A), selbstrückstellende
  Sicherung 3 A und 470-µF-Puffer sind auf dem Shield. Servos können beim Anlaufen kurzzeitig > 1 A ziehen.
* **GND** von Netzteil und UNO Q sind auf dem Shield verbunden (zwingend nötig für die Signale).

### Motor-Erkennung über A0 (Sonderpunkt 1)

Mit JP1 in Stellung **DIV–A0** misst A0 die Motor-Versorgung über einen Teiler (15 kΩ / 33 kΩ, geklemmt
mit BAT54S auf 3,3 V). Liegt keine Motor-Versorgung an, ist A0 ≈ 0 V → die Firmware schaltet in den
Modus „nur Erkennung, keine Bewegung“. Die Schwellen sind: > 2,6 V = angeschlossen, < 2,0 V = nicht
angeschlossen (Hysterese, 200 ms entprellt).

### Schrittmotor 28BYJ-48

Halbschritt-Betrieb (4096 Halbschritte ≈ 1 Umdrehung), max. 500 Halbschritte/s (einstellbar). Im
Stillstand werden die Spulen nach 0,4 s abgeschaltet (Strom sparen, Getriebe hält die Position).
Position 0 ist der Kontaktschalter. Fährt die Referenzfahrt vom Schalter **weg**, auf der
Kalibrierseite „Drehrichtung umkehren“ setzen.

### MPU6050 – Einbaulage

Die Firmware meldet die Beschleunigung aller Achsen (Webseite → Live → Mikrocontroller). Bei
normaler Montage zeigt die Achse mit ≈ **+1000 mg** nach oben – diese Achse unter
*Einstellungen → MPU-Achse „oben“* eintragen (Standard `z` = Modul liegt flach). Steckt das GY-521
senkrecht in J11, ist es meist `y` oder `-y`. Bei Deckenmontage wird dieser Wert negativ und das
Kamerabild automatisch um 180° gedreht.

### RD-03D

UART 256000 Baud, die Firmware schaltet beim Start in den Multi-Target-Modus (bis 3 Ziele). Zeigt
die Radar-Ansicht Personen seitenverkehrt, unter *Einstellungen → Radar-X-Achse spiegeln* umstellen.

### WS2812B

Die Daten werden über die SPI-Schnittstelle (D11, 5 MHz, 8 SPI-Bits pro LED-Bit) erzeugt, damit das
Timing auch neben dem Servo-Interrupt stimmt. Ohne Pegelwandler funktionieren viele WS2812B mit
3,3 V nicht zuverlässig – der 74AHCT1G125 auf dem Shield löst das.

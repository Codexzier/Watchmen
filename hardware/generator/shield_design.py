# =====================================================================================
#  Watchmen-Shield – Bauteile, Verbindungen (Netze) und Platzierung
# =====================================================================================
#  Diese Datei ist die EINZIGE Quelle für Schaltplan UND Platine. build_shield.py liest
#  sie, erzeugt daraus den KiCad-Schaltplan, prüft die Netzliste und baut die Platine.
#
#  Koordinaten der Platine: Draufsicht, USB-C des UNO Q links, Ursprung = linke obere
#  Ecke der UNO-Kontur, x nach rechts, y nach unten (wie in KiCad), Angaben in mm.
#  Die Stiftleisten-Positionen entsprechen dem UNO-R3-Formfaktor (gleich beim UNO Q).
# =====================================================================================

BOARD_NAME = "watchmen-shield"                                # Projektname (Dateinamen)
BOARD_TITLE = "Watchmen Shield fuer Arduino UNO Q"            # Titel im Schaltplan
BOARD_REV = "1.0"                                             # Revision

# Kontur des UNO R3 / UNO Q (Draufsicht, y nach unten) – Polygon im Uhrzeigersinn
OUTLINE = [(0.0, 0.0), (64.52, 0.0), (66.04, 1.52), (66.04, 12.95), (68.58, 15.49),  # oben, rechts oben (Fase)
           (68.58, 48.26), (66.04, 50.80), (66.04, 53.34), (0.0, 53.34)]  # rechts, unten, links

# Fenster über der LED-Matrix des UNO Q (x1, y1, x2, y2). VOR DER BESTELLUNG am echten
# Board prüfen (1:1-Ausdruck docs/outline_1zu1.pdf auf den UNO Q legen) und ggf. anpassen.
WINDOW = (24.5, 11.0, 61.0, 41.5)                              # Ausschnitt in mm
WINDOW_RADIUS = 1.5                                           # Eckenradius des Ausschnitts

# Befestigungslöcher des UNO-Formfaktors (M3)
HOLES = [(13.97, 50.80), (15.24, 2.54), (66.04, 45.72), (66.04, 17.78)]  # Lochmitten

# Bibliotheks-Kürzel
R0805 = "Resistor_SMD:R_0805_2012Metric"                      # Widerstand 0805 (handlötbar)
C0805 = "Capacitor_SMD:C_0805_2012Metric"                     # Kondensator 0805
HDR = "Connector_PinHeader_2.54mm:PinHeader_1x{n:02d}_P2.54mm_Vertical"  # Stiftleiste 2,54 mm

# -------------------------------------------------------------------------------------
#  Bauteile
#  ref, Symbol, Wert, Footprint, Pins {Nummer: Netz | None (= nicht verbunden)},
#  Schaltplan-Position (x, y), Platinen-Position (x, y, Drehung) – Position = Pad 1 bzw.
#  Bauteilmitte bei SMD-Teilen, Drehung in Grad (gegen den Uhrzeigersinn)
# -------------------------------------------------------------------------------------
PARTS = [
    # ---------------- Stiftleisten zum UNO Q (Stifte nach unten in die UNO-Buchsen) ----
    dict(ref="J1", sym="Connector_Generic:Conn_01x08", value="UNO Power", fp=HDR.format(n=8),
         pins={1: None, 2: None, 3: None, 4: "+3V3", 5: "+5V", 6: "GND", 7: "GND", 8: None},
         sch=(40.64, 50.8), pcb=(27.94, 50.80, 90), labels="NC IOR RST 3V3 5V GND GND VIN"),
    dict(ref="J2", sym="Connector_Generic:Conn_01x06", value="UNO Analog", fp=HDR.format(n=6),
         pins={1: "A0_SENSE", 2: "A1_SERVO1", 3: "A2_SERVO2", 4: None, 5: None, 6: None},
         sch=(40.64, 96.52), pcb=(50.80, 50.80, 90), labels="A0 A1 A2 A3 A4 A5"),
    dict(ref="J3", sym="Connector_Generic:Conn_01x08", value="UNO D0-D7", fp=HDR.format(n=8),
         pins={1: "D0_RX", 2: "D1_TX", 3: "D2_SW", 4: "D3_INT", 5: "D4_IN1", 6: None, 7: None, 8: "D7_IN2"},
         sch=(40.64, 137.16), pcb=(63.50, 2.54, -90), labels="D0 D1 D2 D3 D4 D5 D6 D7"),
    dict(ref="J4", sym="Connector_Generic:Conn_01x10", value="UNO D8-SCL", fp=HDR.format(n=10),
         pins={1: "D8_IN3", 2: "D9_IN4", 3: None, 4: "D11_MOSI", 5: None, 6: None, 7: "GND", 8: None, 9: "SDA", 10: "SCL"},
         sch=(40.64, 185.42), pcb=(41.66, 2.54, -90), labels="D8 D9 D10 D11 D12 D13 GND ARF SDA SCL"),

    # ---------------- Motor-Stromversorgung (extern 5 V) mit Verpolschutz --------------
    dict(ref="J5", sym="Connector:Screw_Terminal_01x02", value="MOTOR 5V",
         fp="TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal",
         pins={1: "VMOT_IN", 2: "GND"}, sch=(109.22, 55.88), pcb=(4.4, 40.0, 90)),
    dict(ref="F1", sym="Device:Polyfuse", value="3A hold", fp="Fuse:Fuse_1812_4532Metric",
         pins={1: "VMOT_IN", 2: "VMOT_F"}, sch=(132.08, 55.88), pcb=(13.0, 22.0, 0)),
    dict(ref="Q1", sym="Transistor_FET:AO3401A", value="AO3401A", fp="Package_TO_SOT_SMD:SOT-23",
         pins={1: "Q1_GATE", 2: "VMOT", 3: "VMOT_F"}, sch=(157.48, 58.42), pcb=(16.6, 25.8, 0)),
    dict(ref="R1", sym="Device:R", value="10k", fp=R0805, pins={1: "Q1_GATE", 2: "GND"},
         sch=(154.94, 81.28), pcb=(19.7, 25.8, 90)),
    dict(ref="C1", sym="Device:C_Polarized", value="470uF 10V", fp="Capacitor_THT:CP_Radial_D8.0mm_P3.50mm",
         pins={1: "VMOT", 2: "GND"}, sch=(182.88, 58.42), pcb=(5.0, 47.6, 0)),
    dict(ref="C2", sym="Device:C", value="100nF", fp=C0805, pins={1: "VMOT", 2: "GND"},
         sch=(198.12, 58.42), pcb=(11.8, 26.4, 90)),
    dict(ref="R2", sym="Device:R", value="1k5", fp=R0805, pins={1: "VMOT", 2: "PWR_LED"},
         sch=(213.36, 58.42), pcb=(19.7, 30.6, 90)),
    dict(ref="D1", sym="Device:LED", value="gruen", fp="LED_SMD:LED_0805_2012Metric",
         pins={1: "GND", 2: "PWR_LED"}, sch=(223.52, 76.2), pcb=(19.7, 34.8, 90)),
    dict(ref="JP2", sym="Jumper:SolderJumper_2_Open", value="5V->VMOT",
         fp="Jumper:SolderJumper-2_P1.3mm_Open_RoundedPad1.0x1.5mm",
         pins={1: "+5V", 2: "VMOT"}, sch=(109.22, 81.28), pcb=(13.4, 37.2, 0)),

    # ---------------- A0: Motoren angeschlossen? ----------------------------------------
    dict(ref="R3", sym="Device:R", value="15k", fp=R0805, pins={1: "VMOT", 2: "SENSE_DIV"},
         sch=(264.16, 60.96), pcb=(63.0, 38.0, 90)),
    dict(ref="R4", sym="Device:R", value="33k", fp=R0805, pins={1: "SENSE_DIV", 2: "GND"},
         sch=(279.4, 60.96), pcb=(65.5, 38.0, 90)),
    dict(ref="D2", sym="Diode:BAT54S", value="BAT54S", fp="Package_TO_SOT_SMD:SOT-23",
         pins={1: "GND", 2: "+3V3", 3: "SENSE_DIV"}, sch=(304.8, 68.58), pcb=(64.6, 41.9, 0)),
    dict(ref="JP1", sym="Jumper:Jumper_3_Open", value="A0 Quelle", fp=HDR.format(n=3),
         pins={1: "SENSE_DIV", 2: "SENSE_SEL", 3: "+3V3"}, sch=(332.74, 68.58), pcb=(55.5, 46.6, 90),
         labels="DIV A0 3V3"),
    dict(ref="R5", sym="Device:R", value="1k", fp=R0805, pins={1: "SENSE_SEL", 2: "A0_SENSE"},
         sch=(355.6, 60.96), pcb=(63.0, 34.2, 90)),
    dict(ref="R6", sym="Device:R", value="100k", fp=R0805, pins={1: "A0_SENSE", 2: "GND"},
         sch=(370.84, 60.96), pcb=(65.5, 34.2, 90)),

    # ---------------- Schrittmotor 28BYJ-48 (ULN2003A auf dem Shield) ------------------
    dict(ref="U1", sym="Transistor_Array:ULN2003A", value="ULN2003A", fp="Package_SO:SOIC-16_3.9x9.9mm_P1.27mm",
         pins={1: "D4_IN1", 2: "D7_IN2", 3: "D8_IN3", 4: "D9_IN4", 5: "GND", 6: "GND", 7: "GND", 8: "GND",
               9: "VMOT", 10: None, 11: None, 12: None, 13: "STEP_4", 14: "STEP_3", 15: "STEP_2", 16: "STEP_1"},
         sch=(119.38, 137.16), pcb=(12.4, 13.4, 180)),
    dict(ref="C3", sym="Device:C", value="100nF", fp=C0805, pins={1: "VMOT", 2: "GND"},
         sch=(149.86, 124.46), pcb=(11.0, 6.4, 0)),
    dict(ref="J6", sym="Connector_Generic:Conn_01x05", value="28BYJ-48",
         fp="Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical",
         pins={1: "STEP_4", 2: "STEP_3", 3: "STEP_2", 4: "STEP_1", 5: "VMOT"}, sch=(175.26, 137.16), pcb=(3.4, 8.2, -90),
         labels="BL PK YE OR RD"),

    # ---------------- Servos ------------------------------------------------------------
    dict(ref="R7", sym="Device:R", value="220", fp=R0805, pins={1: "A1_SERVO1", 2: "SERVO1_SIG"},
         sch=(215.9, 127.0), pcb=(31.4, 46.6, 90)),
    dict(ref="J7", sym="Connector_Generic:Conn_01x03", value="SERVO 1 Kamera", fp=HDR.format(n=3),
         pins={1: "SERVO1_SIG", 2: "VMOT", 3: "GND"}, sch=(241.3, 129.54), pcb=(34.36, 46.6, 90), labels="S + -"),
    dict(ref="R8", sym="Device:R", value="220", fp=R0805, pins={1: "A2_SERVO2", 2: "SERVO2_SIG"},
         sch=(215.9, 165.1), pcb=(42.4, 46.6, 90)),
    dict(ref="J8", sym="Connector_Generic:Conn_01x03", value="SERVO 2 Radar", fp=HDR.format(n=3),
         pins={1: "SERVO2_SIG", 2: "VMOT", 3: "GND"}, sch=(241.3, 167.64), pcb=(45.36, 46.6, 90), labels="S + -"),

    # ---------------- Kontaktschalter (Referenz der Drehachse) ---------------------------
    dict(ref="J9", sym="Connector_Generic:Conn_01x02", value="REF SCHALTER", fp=HDR.format(n=2),
         pins={1: "SW_IN", 2: "GND"}, sch=(309.88, 129.54), pcb=(65.0, 23.6, 0), labels="SW GND"),
    dict(ref="R10", sym="Device:R", value="1k", fp=R0805, pins={1: "SW_IN", 2: "D2_SW"},
         sch=(332.74, 129.54), pcb=(63.0, 30.4, 90)),
    dict(ref="C4", sym="Device:C", value="100nF", fp=C0805, pins={1: "SW_IN", 2: "GND"},
         sch=(284.48, 149.86), pcb=(65.5, 30.4, 90)),

    # ---------------- AI-Thinker RD-03D (UART 256000 Baud) -------------------------------
    dict(ref="J10", sym="Connector_Generic:Conn_01x04", value="RD-03D", fp=HDR.format(n=4),
         pins={1: "+5V", 2: "GND", 3: "RADAR_TX", 4: "RADAR_RX"}, sch=(134.62, 210.82), pcb=(51.8, 6.6, 90),
         labels="5V GND TX RX"),
    dict(ref="R11", sym="Device:R", value="470", fp=R0805, pins={1: "RADAR_TX", 2: "D0_RX"},
         sch=(160.02, 205.74), pcb=(55.0, 9.6, 0)),
    dict(ref="R12", sym="Device:R", value="470", fp=R0805, pins={1: "D1_TX", 2: "RADAR_RX"},
         sch=(180.34, 205.74), pcb=(58.6, 9.6, 0)),
    dict(ref="C5", sym="Device:C", value="10uF", fp=C0805, pins={1: "+5V", 2: "GND"},
         sch=(111.76, 210.82), pcb=(49.0, 6.6, 90)),

    # ---------------- MPU6050 (Modul GY-521 in Buchsenleiste) ----------------------------
    dict(ref="J11", sym="Connector_Generic:Conn_01x08", value="MPU6050 GY-521",
         fp="Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical",
         pins={1: "+3V3", 2: "GND", 3: "SCL", 4: "SDA", 5: None, 6: None, 7: "GND", 8: "D3_INT"},
         sch=(375.92, 132.08), pcb=(26.4, 6.6, 90), labels="VCC GND SCL SDA XDA XCL AD0 INT"),
    dict(ref="C6", sym="Device:C", value="100nF", fp=C0805, pins={1: "+3V3", 2: "GND"},
         sch=(398.78, 132.08), pcb=(47.0, 6.6, 90)),

    # ---------------- WS2812B (Datenleitung über Pegelwandler 3,3 V → 5 V) --------------
    dict(ref="U2", sym="74xGxx:74AHCT1G125", value="74AHCT1G125", fp="Package_TO_SOT_SMD:SOT-23-5",
         pins={1: "GND", 2: "D11_MOSI", 3: "GND", 4: "LED_BUF", 5: "+5V"}, sch=(264.16, 213.36), pcb=(17.6, 43.8, 0)),
    dict(ref="R13", sym="Device:R", value="10k", fp=R0805, pins={1: "D11_MOSI", 2: "GND"},
         sch=(236.22, 228.6), pcb=(13.4, 44.6, 90)),
    dict(ref="R14", sym="Device:R", value="330", fp=R0805, pins={1: "LED_BUF", 2: "LED_DIN"},
         sch=(292.1, 213.36), pcb=(20.3, 47.6, 90)),
    dict(ref="C7", sym="Device:C", value="100nF", fp=C0805, pins={1: "+5V", 2: "GND"},
         sch=(264.16, 241.3), pcb=(17.6, 40.4, 0)),
    dict(ref="J12", sym="Connector_Generic:Conn_01x03", value="WS2812B", fp=HDR.format(n=3),
         pins={1: "+5V", 2: "LED_DIN", 3: "GND"}, sch=(322.58, 215.9), pcb=(23.36, 46.6, 90), labels="5V DIN GND"),

    # ---------------- Befestigungslöcher -----------------------------------------------
    dict(ref="H1", sym="Mechanical:MountingHole", value="M3", fp="MountingHole:MountingHole_3.2mm_M3",
         pins={}, sch=(365.76, 205.74), pcb=(HOLES[0][0], HOLES[0][1], 0)),
    dict(ref="H2", sym="Mechanical:MountingHole", value="M3", fp="MountingHole:MountingHole_3.2mm_M3",
         pins={}, sch=(378.46, 205.74), pcb=(HOLES[1][0], HOLES[1][1], 0)),
    dict(ref="H3", sym="Mechanical:MountingHole", value="M3", fp="MountingHole:MountingHole_3.2mm_M3",
         pins={}, sch=(365.76, 218.44), pcb=(HOLES[2][0], HOLES[2][1], 0)),
    dict(ref="H4", sym="Mechanical:MountingHole", value="M3", fp="MountingHole:MountingHole_3.2mm_M3",
         pins={}, sch=(378.46, 218.44), pcb=(HOLES[3][0], HOLES[3][1], 0)),
]

# Netzklassen (Leiterbahnbreiten) – breite Bahnen für Motorströme
NETCLASSES = {
    "Power": dict(track=0.7, clearance=0.2, via=0.8, drill=0.4,        # Motor-Versorgung (ca. 2 A, Spitzen mehr)
                  nets=["VMOT", "VMOT_IN", "VMOT_F"]),
    "Supply": dict(track=0.3, clearance=0.2, via=0.6, drill=0.3,       # 5 V für Radar und LEDs (ca. 0,3 A)
                   nets=["+5V"]),
    "Motor": dict(track=0.3, clearance=0.2, via=0.6, drill=0.3,        # Spulen 28BYJ-48 (ca. 200 mA)
                  nets=["STEP_1", "STEP_2", "STEP_3", "STEP_4"]),
    "Default": dict(track=0.25, clearance=0.2, via=0.6, drill=0.3, nets=[]),  # Signale (GND nur über Kupferflächen + Vias)
}

# Netze, die im Schaltplan ein PWR_FLAG bekommen (für die elektrische Regelprüfung)
PWR_FLAG_NETS = ["GND", "+5V", "+3V3", "VMOT_IN"]

# Erklärungstexte im Schaltplan (Position, Text)
SCH_NOTES = [
    ((20.32, 33.02), "Stiftleisten UNO Q"),
    ((96.52, 33.02), "Motor-Versorgung 5 V extern, Verpolschutz (Q1), Sicherung (F1)"),
    ((256.54, 33.02), "A0 = 3,3 V wenn Motoren versorgt (JP1: AUTO=Teiler / 3V3=immer)"),
    ((96.52, 109.22), "Schrittmotor 28BYJ-48 (ULN2003A, JST-XH-Stecker)"),
    ((203.2, 109.22), "Servos (Signal mit Schutzwiderstand)"),
    ((271.78, 109.22), "Referenzschalter (Pull-up im MCU)"),
    ((355.6, 109.22), "MPU6050 Modul (I2C)"),
    ((96.52, 190.5), "RD-03D Radar (UART D0/D1, 256000 Baud)"),
    ((223.52, 190.5), "WS2812B: SPI-MOSI (D11) -> Pegelwandler 5 V"),
    ((355.6, 195.58), "Befestigungsloecher"),
]

# Namen der Steckverbinder auf dem Bestückungsdruck oben: Text, (x, y), Drehung
SILK_NAMES = [
    ("MPU6050 (GY-521)", (35.3, 4.7), 0),
    ("RD-03D", (55.6, 4.7), 0),
    ("REF", (65.0, 21.6), 0),
    ("WS2812B", (25.9, 48.75), 0),
    ("SERVO1", (36.9, 48.75), 0),
    ("SERVO2", (47.9, 48.75), 0),
    ("A0-QUELLE", (58.0, 48.75), 0),
    ("MOTOR 5V", (10.3, 37.4), 90),
    ("+", (8.6, 40.0), 0),
    ("STEPPER", (7.4, 13.2), 90),
    ("WATCHMEN v1.0", (6.6, 3.0), 0),
]

# Stiftleisten zum UNO, deren Pin-Namen auf der Unterseite gedruckt werden (oben ist kein Platz)
BACK_LABELS = ["J1", "J2", "J3", "J4"]

# Löcher, die sehr nah an Stiftleisten liegen (UNO-Formfaktor): ohne Bauteil-Sperrfläche,
# hier Kunststoff-/M2.5-Schrauben verwenden (siehe Doku)
TIGHT_HOLES = ["H2", "H3"]

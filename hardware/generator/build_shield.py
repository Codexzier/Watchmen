#!/usr/bin/env python3
# =====================================================================================
#  Watchmen-Shield – erzeugt das komplette KiCad-Projekt aus shield_design.py
# =====================================================================================
#  Schritte:
#   1. Schaltplan (.kicad_sch) mit Symbolen aus den installierten KiCad-Bibliotheken
#   2. Netzliste mit kicad-cli exportieren und gegen shield_design.py prüfen
#   3. Platine (.kicad_pcb) über die pcbnew-Python-API: Kontur, Fenster, Bauteile, Netze
#   4. Autorouting mit Freerouting (Specctra DSN → SES), Masseflächen, DRC
#   5. Fertigungsdaten (Gerber, Bohrdaten, Bestückung, Stückliste) und Doku (PDF/SVG)
#
#  Aufruf (KiCad 7 mit Python-Modul "pcbnew", Java 25 und freerouting.jar nötig):
#     /usr/bin/python3 hardware/generator/build_shield.py --freerouting /pfad/freerouting.jar
# =====================================================================================

import argparse                                               # Kommandozeilen-Parameter
import copy                                                   # tiefe Kopien von Listen
import csv                                                    # Stückliste als CSV
import json                                                   # Projektdatei (.kicad_pro)
import os                                                     # Pfade
import re                                                     # reguläre Ausdrücke (Parser)
import shutil                                                 # Programme suchen, Dateien kopieren
import subprocess                                             # kicad-cli und Java aufrufen
import sys                                                    # Programmende mit Fehlercode
import uuid                                                   # eindeutige IDs für KiCad-Objekte
from collections import defaultdict                           # Wörterbuch mit Standardwert

HERE = os.path.dirname(os.path.abspath(__file__))             # Ordner dieses Skripts
sys.path.insert(0, HERE)                                      # damit shield_design gefunden wird
import shield_design as D                                     # noqa: E402  Bauteile und Netze

OUT = os.path.abspath(os.path.join(HERE, "..", D.BOARD_NAME))  # Zielordner des KiCad-Projekts
SYMBOL_DIR = "/usr/share/kicad/symbols"                       # installierte Symbolbibliotheken
FOOTPRINT_DIR = "/usr/share/kicad/footprints"                 # installierte Footprint-Bibliotheken
ORIGIN = (60.0, 40.0)                                         # Lage der Platine auf dem Zeichenblatt [mm]
STUB = 2.54                                                   # Länge der Anschluss-Striche im Schaltplan


def uid():                                                    # neue eindeutige ID
    return str(uuid.uuid4())                                  # zufällige UUID als Text


# ------------------------------------------------------------------ S-Ausdrücke -------
TOKEN = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+')       # Klammern, Texte in "", Wörter


def sexpr_parse(text):                                        # liest KiCad-S-Ausdrücke in Listen
    stack = [[]]                                              # Stapel offener Listen
    for m in TOKEN.finditer(text):                            # jedes Token
        t = m.group(0)                                        # Token-Text
        if t == "(":                                          # neue Liste beginnt
            stack.append([])                                  # auf den Stapel
        elif t == ")":                                        # Liste endet
            node = stack.pop()                                # fertige Liste holen
            stack[-1].append(node)                            # an übergeordnete Liste hängen
        else:                                                 # Wort oder Text
            stack[-1].append(t)                               # unverändert übernehmen
    return stack[0]                                           # oberste Ebene zurückgeben


def sexpr_dump(node, indent=0):                               # schreibt Listen als S-Ausdruck
    if not isinstance(node, list):                            # einfaches Element
        return node                                           # unverändert
    simple = all(not isinstance(x, list) for x in node)       # nur einfache Elemente?
    if simple:                                                # dann in eine Zeile
        return "(" + " ".join(node) + ")"                     # z.B. (at 1 2 0)
    pad = "  " * (indent + 1)                                 # Einrückung der Kinder
    parts = [sexpr_dump(x, indent + 1) for x in node]         # Kinder schreiben
    head = []                                                 # einfache Elemente am Anfang
    while parts and not isinstance(node[len(head)], list):    # führende Wörter sammeln
        head.append(parts.pop(0))                             # in den Kopf übernehmen
    return "(" + " ".join(head) + "".join("\n" + pad + p for p in parts) + ")"  # mehrzeilig


def q(text):                                                  # Text in Anführungszeichen setzen
    return '"' + str(text).replace("\\", "\\\\").replace('"', '\\"') + '"'  # mit Maskierung


def unq(token):                                               # Anführungszeichen entfernen
    return token[1:-1] if token.startswith('"') else token    # nur wenn vorhanden


def find(node, key):                                          # erstes Kind mit Namen key
    for x in node:                                            # alle Kinder
        if isinstance(x, list) and x and x[0] == key:         # passender Name
            return x                                          # zurückgeben
    return None                                               # nicht gefunden


def find_all(node, key):                                      # alle Kinder mit Namen key (rekursiv)
    out = []                                                  # Ergebnisliste
    for x in node:                                            # alle Kinder
        if isinstance(x, list):                               # nur Listen
            if x and x[0] == key:                             # passender Name
                out.append(x)                                 # merken
            out.extend(find_all(x, key))                      # auch in der Tiefe suchen
    return out                                                # Liste zurückgeben


# ------------------------------------------------------------------ Symbole ----------
_LIB_CACHE = {}                                               # bereits gelesene Bibliotheken


def load_lib(lib):                                            # liest eine Symbolbibliothek
    if lib not in _LIB_CACHE:                                 # noch nicht gelesen?
        with open(os.path.join(SYMBOL_DIR, lib + ".kicad_sym"), encoding="utf-8") as f:  # Datei öffnen
            root = sexpr_parse(f.read())[0]                   # parsen
        _LIB_CACHE[lib] = {unq(n[1]): n for n in root if isinstance(n, list) and n[0] == "symbol"}  # nach Namen
    return _LIB_CACHE[lib]                                    # Wörterbuch zurückgeben


def lib_symbol(lib_id):                                       # Symbol für den Schaltplan (flach, umbenannt)
    lib, name = lib_id.split(":")                             # Bibliothek und Name trennen
    syms = load_lib(lib)                                      # Bibliothek laden
    node = copy.deepcopy(syms[name])                          # Symbol kopieren
    ext = find(node, "extends")                               # abgeleitetes Symbol?
    if ext:                                                   # ja: Elternsymbol einbauen
        parent = lib_symbol(lib + ":" + unq(ext[1]))          # Eltern (rekursiv, bereits flach)
        parent_name = unq(parent[1]).split(":")[1]            # Name des Elternsymbols
        props = {unq(p[1]): p for p in node if isinstance(p, list) and p[0] == "property"}  # eigene Eigenschaften
        out = ["symbol", q(lib_id)]                           # neues Symbol
        for child in parent[2:]:                              # Inhalt der Eltern übernehmen
            if isinstance(child, list) and child[0] == "property" and unq(child[1]) in props:  # überschrieben?
                out.append(props.pop(unq(child[1])))          # eigene Eigenschaft verwenden
            elif isinstance(child, list) and child[0] == "symbol":  # Unter-Symbol (Grafik, Pins)
                child = copy.deepcopy(child)                  # kopieren
                child[1] = q(name + unq(child[1])[len(parent_name):])  # umbenennen (Eltern_0_1 → Name_0_1)
                out.append(child)                             # übernehmen
            else:                                             # sonstiges
                out.append(child)                             # übernehmen
        out.extend(props.values())                            # restliche eigene Eigenschaften
        return out                                            # fertig
    node[1] = q(lib_id)                                       # Namen mit Bibliothek versehen
    return node                                               # zurückgeben


def symbol_pins(sym):                                         # Pins eines (flachen) Symbols
    pins = {}                                                 # Nummer → (x, y, Winkel)
    for p in find_all(sym, "pin"):                            # alle Pin-Einträge
        at = find(p, "at")                                    # Position
        num = find(p, "number")                               # Nummer
        if at is None or num is None:                         # keine Pin-Definition
            continue                                          # überspringen
        pins[unq(num[1])] = (float(at[1]), float(at[2]), int(float(at[3])))  # speichern
    return pins                                               # zurückgeben


# ------------------------------------------------------------------ Schaltplan -------
class Schematic:                                              # sammelt alle Elemente des Schaltplans
    def __init__(self):                                       # Konstruktor
        self.root_uuid = uid()                                # ID des Hauptblatts
        self.lib_symbols = {}                                 # verwendete Bibliothekssymbole
        self.items = []                                       # Symbole, Drähte, Labels ...
        self.pwr_count = 0                                    # Zähler für #PWR-Referenzen
        self.flg_count = 0                                    # Zähler für #FLG-Referenzen
        self.symbol_uuids = {}                                # Referenz → UUID (für die Platine)

    def _effects(self, size=1.27, hide=False, justify=None):  # Text-Eigenschaften
        e = ["effects", ["font", ["size", str(size), str(size)]]]  # Schriftgröße
        if justify:                                           # Ausrichtung
            e.append(["justify"] + justify.split())           # z.B. left bottom
        if hide:                                              # versteckt
            e.append("hide")                                  # Kennzeichen
        return e                                              # zurückgeben

    def add_symbol(self, lib_id, ref, value, x, y, footprint="", angle=0):  # Symbol platzieren
        if lib_id not in self.lib_symbols:                    # Symbol noch nicht im Schaltplan?
            self.lib_symbols[lib_id] = lib_symbol(lib_id)     # aus der Bibliothek holen
        sym = self.lib_symbols[lib_id]                        # Symboldefinition
        pins = symbol_pins(sym)                               # Pins
        ys = [-p[1] for p in pins.values()] or [0.0]          # Pin-Höhen (Schaltplan-Richtung)
        top, bottom = min(ys), max(ys)                        # Ausdehnung nach oben/unten
        vertical_only = pins and all(p[2] in (90, 270) for p in pins.values())  # nur Pins oben/unten (z.B. R, C)?
        u = uid()                                             # ID des Symbols
        self.symbol_uuids[ref] = u                            # merken
        hide_ref = ref.startswith("#")                        # Netzsymbole: Referenz versteckt
        if vertical_only and len(pins) <= 2:                  # zweipoliges stehendes Bauteil
            ref_at = (x + 2.54, y - 1.27, "left")             # Referenz rechts oben
            val_at = (x + 2.54, y + 1.27, "left")             # Wert rechts unten
        elif ref.startswith("#"):                             # Netzsymbol
            ref_at = (x, y + 3.81, None)                      # Referenz (versteckt)
            val_at = (x, y - 3.81 if angle == 0 else y + 3.81, None)  # Wert neben dem Symbol
        else:                                                 # sonstige Symbole
            ref_at = (x, y + top - 3.81, None)                # Referenz oben
            val_at = (x, y + bottom + 3.81, None)             # Wert unten
        node = ["symbol", ["lib_id", q(lib_id)], ["at", f"{x:.2f}", f"{y:.2f}", str(angle)], ["unit", "1"],  # Kopf
                ["in_bom", "no" if hide_ref else "yes"], ["on_board", "no" if hide_ref else "yes"],  # Stückliste/Platine
                ["dnp", "no"], ["uuid", u],                   # nicht "nicht bestücken", ID
                ["property", q("Reference"), q(ref), ["at", f"{ref_at[0]:.2f}", f"{ref_at[1]:.2f}", "0"],  # Referenz
                 self._effects(hide=hide_ref, justify=ref_at[2])],
                ["property", q("Value"), q(value), ["at", f"{val_at[0]:.2f}", f"{val_at[1]:.2f}", "0"],  # Wert
                 self._effects(justify=val_at[2], hide=ref.startswith("#FLG"))],
                ["property", q("Footprint"), q(footprint), ["at", f"{x:.2f}", f"{y:.2f}", "0"], self._effects(hide=True)],  # Footprint
                ["property", q("Datasheet"), q("~"), ["at", f"{x:.2f}", f"{y:.2f}", "0"], self._effects(hide=True)]]  # Datenblatt
        for num in pins:                                      # jeder Pin bekommt eine ID
            node.append(["pin", q(num), ["uuid", uid()]])     # Pin-Eintrag
        node.append(["instances", ["project", q(D.BOARD_NAME),  # Zuordnung zum Projekt
                                   ["path", q("/" + self.root_uuid), ["reference", q(ref)], ["unit", "1"]]]])
        self.items.append(node)                               # zum Schaltplan
        return pins                                           # Pins zurückgeben (für Anschlüsse)

    def wire(self, x1, y1, x2, y2):                           # Draht zeichnen
        self.items.append(["wire", ["pts", ["xy", f"{x1:.2f}", f"{y1:.2f}"], ["xy", f"{x2:.2f}", f"{y2:.2f}"]],  # Endpunkte
                           ["stroke", ["width", "0"], ["type", "default"]], ["uuid", uid()]])

    def label(self, name, x, y, angle):                       # Netzname (lokales Label)
        just = {0: "left bottom", 90: "left bottom", 180: "right bottom", 270: "right bottom"}[angle]  # Ausrichtung
        self.items.append(["label", q(name), ["at", f"{x:.2f}", f"{y:.2f}", str(angle)], ["fields_autoplaced"],
                           self._effects(justify=just), ["uuid", uid()]])

    def global_label(self, name, x, y, angle):                # globales Label (z.B. GND an einer Steckerleiste)
        just = "right" if angle == 180 else "left"            # Ausrichtung
        self.items.append(["global_label", q(name), ["shape", "passive"], ["at", f"{x:.2f}", f"{y:.2f}", str(angle)],
                           ["fields_autoplaced"], self._effects(justify=just), ["uuid", uid()],
                           ["property", q("Intersheetrefs"), q("${INTERSHEET_REFS}"), ["at", f"{x:.2f}", f"{y:.2f}", "0"],
                            self._effects(justify=just, hide=True)]])

    def no_connect(self, x, y):                               # Kennzeichen "nicht verbunden"
        self.items.append(["no_connect", ["at", f"{x:.2f}", f"{y:.2f}"], ["uuid", uid()]])

    def text(self, text, x, y, size=2.0):                     # Überschrift/Hinweis
        self.items.append(["text", q(text), ["at", f"{x:.2f}", f"{y:.2f}", "0"],
                           self._effects(size=size, justify="left bottom"), ["uuid", uid()]])

    def power(self, net, x, y, direction=(0, 1)):            # Netzsymbol (GND, +5V, +3V3) in Leitungsrichtung
        self.pwr_count += 1                                   # Zähler erhöhen
        if net == "GND":                                      # Masse zeigt von Haus aus nach unten
            angle = {(0, 1): 0, (0, -1): 180, (-1, 0): 270, (1, 0): 90}[direction]  # Drehung passend zur Richtung
        else:                                                 # +5V/+3V3 zeigen von Haus aus nach oben
            angle = {(0, -1): 0, (0, 1): 180, (-1, 0): 90, (1, 0): 270}[direction]  # Drehung passend zur Richtung
        self.add_symbol("power:" + net, f"#PWR{self.pwr_count:03d}", net, x, y, angle=angle)  # Symbol setzen

    def pwr_flag(self, x, y):                                 # PWR_FLAG (Quelle für die Regelprüfung)
        self.flg_count += 1                                   # Zähler erhöhen
        self.add_symbol("power:PWR_FLAG", f"#FLG{self.flg_count:02d}", "PWR_FLAG", x, y)  # Symbol setzen

    def connect_pin(self, net, px, py, angle):                # Pin mit einem Netz verbinden
        out = {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}[angle]  # Richtung nach außen (Schaltplan)
        ex, ey = px + out[0] * STUB, py + out[1] * STUB       # Ende des Anschluss-Strichs
        self.wire(px, py, ex, ey)                             # Strich zeichnen
        if net is None:                                       # nicht verbunden
            self.no_connect(px, py)                           # Kreuz am Pin
            return                                            # fertig
        if net in ("GND", "+5V", "+3V3") and out[0] == 0:     # Versorgungsnetz an senkrechtem Strich
            self.power(net, ex, ey, out)                      # Netzsymbol am Strich-Ende, in Leitungsrichtung
            return                                            # fertig
        if net in ("GND", "+5V", "+3V3"):                     # Versorgungsnetz an waagerechtem Strich (z.B. Steckerleiste)
            self.global_label(net, ex, ey, 180 if out[0] < 0 else 0)  # globales Label (verbindet mit dem Netzsymbol-Netz)
            return                                            # fertig
        lab_angle = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}[out]  # Label-Richtung
        self.label(net, ex, ey, lab_angle)                    # Netzname am Strich-Ende

    def save(self, path):                                     # Schaltplan-Datei schreiben
        root = ["kicad_sch", ["version", "20230121"], ["generator", "eeschema"], ["uuid", self.root_uuid],  # Kopf
                ["paper", q("A3")],                           # Blattgröße
                ["title_block", ["title", q(D.BOARD_TITLE)], ["date", q("2026-10-02")], ["rev", q(D.BOARD_REV)],  # Schriftfeld
                 ["company", q("Watchmen")], ["comment", "1", q("Generiert mit hardware/generator/build_shield.py")]],
                ["lib_symbols"] + list(self.lib_symbols.values())]  # verwendete Symbole
        root += self.items                                    # alle Elemente
        root.append(["sheet_instances", ["path", q("/"), ["page", q("1")]]])  # Seitennummer
        with open(path, "w", encoding="utf-8") as f:          # Datei öffnen
            f.write(sexpr_dump(root) + "\n")                  # schreiben


def build_schematic(path):                                    # erzeugt den Schaltplan aus shield_design
    sch = Schematic()                                         # neuer Schaltplan
    for (x, y), text in D.SCH_NOTES:                          # Überschriften der Funktionsblöcke
        sch.text(text, x, y)                                  # setzen
    for part in D.PARTS:                                      # jedes Bauteil
        x, y = part["sch"]                                    # Position im Schaltplan
        pins = sch.add_symbol(part["sym"], part["ref"], part["value"], x, y, part["fp"])  # Symbol setzen
        for num, (px, py, ang) in pins.items():               # jeder Pin des Symbols
            net = part["pins"].get(int(num)) if num.isdigit() else None  # gewünschtes Netz
            if int(num) not in part["pins"]:                  # Pin nicht in der Beschreibung
                raise SystemExit(f"{part['ref']}: Pin {num} fehlt in shield_design.py")  # Fehler
            sch.connect_pin(net, x + px, y - py, ang)         # anschließen (Symbol-y zeigt nach oben)
    fx, fy = 20.32, 266.7                                     # Ort der PWR_FLAGs
    sch.text("Versorgungs-Kennzeichnung (PWR_FLAG)", fx, fy - 7.62, 1.5)  # Überschrift
    for i, net in enumerate(D.PWR_FLAG_NETS):                 # jedes zu kennzeichnende Netz
        x = fx + i * 20.32                                    # nebeneinander
        sch.wire(x, fy, x + 5.08, fy)                         # kurzer Draht
        if net in ("GND", "+5V", "+3V3"):                     # Versorgungsnetz
            sch.power(net, x, fy, (0, 1) if net == "GND" else (0, -1))  # Netzsymbol links
        else:                                                 # sonstiges Netz
            sch.label(net, x, fy, 180)                        # Label links
        sch.pwr_flag(x + 5.08, fy)                            # PWR_FLAG rechts
    sch.save(path)                                            # Datei schreiben
    return sch                                                # Schaltplan-Objekt zurückgeben


# ------------------------------------------------------------------ Netzliste --------
def run(cmd, **kw):                                           # Programm aufrufen und Fehler melden
    res = subprocess.run(cmd, capture_output=True, text=True, **kw)  # ausführen
    if res.returncode != 0:                                   # Fehler?
        print(res.stdout, res.stderr)                         # Ausgabe zeigen
        raise SystemExit(f"Befehl fehlgeschlagen: {' '.join(cmd)}")  # abbrechen
    return res                                                # Ergebnis zurückgeben


def read_netlist(sch_path, net_path):                         # exportiert und liest die Netzliste
    run(["kicad-cli", "sch", "export", "netlist", "--output", net_path, sch_path])  # mit kicad-cli exportieren
    with open(net_path, encoding="utf-8") as f:               # Datei öffnen
        root = sexpr_parse(f.read())[0]                       # parsen
    nets = {}                                                 # (Referenz, Pin) → Netzname
    members = defaultdict(list)                               # Netzname → Anschlüsse
    for net in find_all(root, "net"):                         # jedes Netz
        name = unq(find(net, "name")[1])                      # Netzname
        for node in [n for n in net if isinstance(n, list) and n[0] == "node"]:  # jeder Anschluss
            ref = unq(find(node, "ref")[1])                   # Bauteil
            pin = unq(find(node, "pin")[1])                   # Pin
            nets[(ref, pin)] = name                           # speichern
            members[name].append((ref, pin))                  # speichern
    return nets, members                                      # zurückgeben


def verify_netlist(nets, members):                            # vergleicht Netzliste und Entwurf
    errors = []                                               # gefundene Fehler
    for part in D.PARTS:                                      # jedes Bauteil
        for num, want in part["pins"].items():                # jeder Pin
            got = nets.get((part["ref"], str(num)))           # tatsächliches Netz
            if want is None:                                  # soll unverbunden sein
                if got is not None and len(members[got]) > 1:  # ist aber verbunden
                    errors.append(f"{part['ref']}.{num}: sollte frei sein, hängt an {got}")  # Fehler
            elif got is None or got.lstrip("/") != want:      # falsches Netz
                errors.append(f"{part['ref']}.{num}: erwartet {want}, gefunden {got}")  # Fehler
    for name, nodes in members.items():                       # jedes Netz
        if not name.startswith("unconnected") and len(nodes) < 2 and not name.startswith("Net-("):  # nur 1 Anschluss
            errors.append(f"Netz {name} hat nur einen Anschluss: {nodes}")  # Warnung als Fehler
    return errors                                             # Liste zurückgeben


# ------------------------------------------------------------------ Projektdatei -----
IGNORED_DRC = {"lib_footprint_issues", "lib_footprint_mismatch", "silk_overlap", "silk_edge_clearance"}  # nur Schönheit


def net_pattern(net):                                         # Netzname wie in der Netzliste (lokale Labels mit "/")
    return net if net in ("GND", "+5V", "+3V3") else "/" + net  # Versorgungsnetze ohne Schrägstrich


def patch_project(path):                                      # ergänzt die von KiCad geschriebene .kicad_pro
    with open(path, encoding="utf-8") as f:                   # Datei lesen
        pro = json.load(f)                                    # JSON laden
    ns = pro["net_settings"]                                  # Netz-Einstellungen
    template = dict(ns["classes"][0])                         # Default-Klasse als Vorlage
    classes, patterns = [], []                                # neue Klassen und Zuordnungen
    for name, nc in D.NETCLASSES.items():                     # jede Klasse aus dem Entwurf
        c = dict(template)                                    # Vorlage kopieren
        c.update({"name": name, "clearance": nc["clearance"], "track_width": nc["track"],  # Werte setzen
                  "via_diameter": nc["via"], "via_drill": nc["drill"]})
        classes.append(c)                                     # übernehmen
        patterns += [{"netclass": name, "pattern": net_pattern(n)} for n in nc["nets"]]  # Netze zuordnen
    ns["classes"] = classes                                   # Klassen ersetzen
    ns["netclass_patterns"] = patterns                        # Zuordnungen ersetzen
    ds = pro["board"]["design_settings"]                      # Entwurfsregeln
    ds["rules"].update({"min_text_height": 0.7, "min_text_thickness": 0.1, "min_copper_edge_clearance": 0.3,  # Grenzwerte
                        "min_track_width": 0.15})
    for key in IGNORED_DRC:                                   # rein optische Prüfungen
        ds["rule_severities"][key] = "ignore"                 # in der KiCad-Oberfläche ausblenden
    with open(path, "w", encoding="utf-8") as f:              # Datei schreiben
        json.dump(pro, f, indent=2)                           # speichern


# ------------------------------------------------------------------ Platine ----------
def mm(v):                                                    # mm → KiCad-Einheit (nm)
    import pcbnew                                             # erst hier (nur mit KiCad verfügbar)
    return pcbnew.FromMM(v)                                   # umrechnen


def pt(x, y):                                                 # Punkt auf dem Blatt (mit Ursprung)
    import pcbnew                                             # KiCad-Modul
    return pcbnew.VECTOR2I(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))  # Vektor in nm


def add_segment(board, layer, x1, y1, x2, y2, width=0.1):     # Linie auf eine Lage zeichnen
    import pcbnew                                             # KiCad-Modul
    s = pcbnew.PCB_SHAPE(board)                               # neue Form
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)                        # Linie
    s.SetStart(pt(x1, y1))                                    # Anfang
    s.SetEnd(pt(x2, y2))                                      # Ende
    s.SetLayer(layer)                                         # Lage
    s.SetWidth(mm(width))                                     # Strichbreite
    board.Add(s)                                              # zur Platine


def add_arc(board, layer, cx, cy, sx, sy, angle, width=0.1):  # Bogen (Mitte, Start, Winkel)
    import pcbnew                                             # KiCad-Modul
    s = pcbnew.PCB_SHAPE(board)                               # neue Form
    s.SetShape(pcbnew.SHAPE_T_ARC)                            # Bogen
    s.SetCenter(pt(cx, cy))                                   # Mittelpunkt
    s.SetStart(pt(sx, sy))                                    # Startpunkt
    s.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T), True)  # Winkel
    s.SetLayer(layer)                                         # Lage
    s.SetWidth(mm(width))                                     # Strichbreite
    board.Add(s)                                              # zur Platine


def add_text(board, text, x, y, size=0.8, layer=None, angle=0, bold=False):  # Text auf die Platine
    import pcbnew                                             # KiCad-Modul
    t = pcbnew.PCB_TEXT(board)                                # neuer Text
    t.SetText(text)                                           # Inhalt
    t.SetPosition(pt(x, y))                                   # Position
    t.SetLayer(pcbnew.F_SilkS if layer is None else layer)    # Lage (Standard: Bestückungsdruck oben)
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))        # Schriftgröße
    t.SetTextThickness(mm(max(0.12, size * 0.15)))            # Strichstärke
    t.SetTextAngle(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))  # Drehung
    t.SetBold(bold)                                           # fett
    board.Add(t)                                              # zur Platine
    return t                                                  # Text zurückgeben


def draw_outline(board):                                      # Platinenkontur und Fenster
    import pcbnew                                             # KiCad-Modul
    pts = D.OUTLINE + [D.OUTLINE[0]]                          # Polygon schließen
    for (x1, y1), (x2, y2) in zip(pts, pts[1:]):              # jede Kante
        add_segment(board, pcbnew.Edge_Cuts, x1, y1, x2, y2, 0.1)  # auf Edge.Cuts
    x1, y1, x2, y2 = D.WINDOW                                 # Fenster
    r = D.WINDOW_RADIUS                                       # Eckenradius
    add_segment(board, pcbnew.Edge_Cuts, x1 + r, y1, x2 - r, y1)  # oben
    add_segment(board, pcbnew.Edge_Cuts, x2, y1 + r, x2, y2 - r)  # rechts
    add_segment(board, pcbnew.Edge_Cuts, x2 - r, y2, x1 + r, y2)  # unten
    add_segment(board, pcbnew.Edge_Cuts, x1, y2 - r, x1, y1 + r)  # links
    add_arc(board, pcbnew.Edge_Cuts, x1 + r, y1 + r, x1, y1 + r, 90)  # Ecke links oben
    add_arc(board, pcbnew.Edge_Cuts, x2 - r, y1 + r, x2 - r, y1, 90)  # Ecke rechts oben
    add_arc(board, pcbnew.Edge_Cuts, x2 - r, y2 - r, x2, y2 - r, 90)  # Ecke rechts unten
    add_arc(board, pcbnew.Edge_Cuts, x1 + r, y2 - r, x1 + r, y2, 90)  # Ecke links unten


def zone(board, net, layer, polygon, priority=0):             # Kupferfläche anlegen
    import pcbnew                                             # KiCad-Modul
    z = pcbnew.ZONE(board)                                    # neue Fläche
    z.SetLayer(layer)                                         # Lage
    z.SetNet(net)                                             # Netz
    z.SetAssignedPriority(priority)                           # Priorität
    z.SetLocalClearance(mm(0.3))                              # Abstand zu fremden Netzen
    z.SetMinThickness(mm(0.25))                               # Mindestbreite der Füllung
    z.SetThermalReliefGap(mm(0.4))                            # Wärmefalle: Spalt
    z.SetThermalReliefSpokeWidth(mm(0.5))                     # Wärmefalle: Stegbreite
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)        # Pads über Wärmefallen anbinden
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)  # abgetrennte Inseln entfernen
    outline = z.Outline()                                     # Umriss
    outline.NewOutline()                                      # neue Kontur
    for x, y in polygon:                                      # jeder Punkt
        outline.Append(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))  # anhängen
    board.Add(z)                                              # zur Platine
    return z                                                  # Fläche zurückgeben


def build_pcb(pcb_path, sch, nets):                           # erzeugt die Platine
    import pcbnew                                             # KiCad-Modul
    board = pcbnew.LoadBoard(pcb_path)                        # leere Platine mit Projekt (Netzklassen) laden
    ds = board.GetDesignSettings()                            # Entwurfsregeln
    ds.SetCopperLayerCount(2)                                 # zweilagig
    ds.m_CopperEdgeClearance = mm(0.3)                        # Abstand Kupfer ↔ Rand
    ds.m_MinClearance = mm(0.2)                               # Mindestabstand
    ds.m_TrackMinWidth = mm(0.15)                             # Mindest-Bahnbreite (Verjüngung an IC-Pins)
    ds.m_ViasMinSize = mm(0.5)                                # Mindest-Durchkontaktierung
    ds.m_MinThroughDrill = mm(0.3)                            # Mindest-Bohrung
    ds.m_MinSilkTextHeight = mm(0.7)                          # kleinste Schrifthöhe Bestückungsdruck
    ds.m_MinSilkTextThickness = mm(0.1)                       # kleinste Strichstärke
    for name, nc in D.NETCLASSES.items():                     # Netzklassen anlegen
        cls = pcbnew.NETCLASS(name)                           # neue Klasse
        cls.SetTrackWidth(mm(nc["track"]))                    # Bahnbreite
        cls.SetClearance(mm(nc["clearance"]))                 # Abstand
        cls.SetViaDiameter(mm(nc["via"]))                     # Via-Durchmesser
        cls.SetViaDrill(mm(nc["drill"]))                      # Via-Bohrung
        if name == "Default":                                 # Standardklasse
            ds.m_NetSettings.m_DefaultNetClass = cls          # ersetzen (nicht zusätzlich eintragen)
        else:                                                 # eigene Klasse
            ds.m_NetSettings.m_NetClasses[name] = cls         # eintragen
    netinfo = {}                                              # Netzname → KiCad-Netz
    for name in sorted(set(nets.values())):                   # alle Netze der Netzliste
        if name.startswith("unconnected"):                    # freie Pins bekommen kein Netz
            continue                                          # überspringen
        n = pcbnew.NETINFO_ITEM(board, name)                  # neues Netz
        board.Add(n)                                          # zur Platine
        netinfo[name] = n                                     # merken
    for part in D.PARTS:                                      # jedes Bauteil
        lib, name = part["fp"].split(":")                     # Footprint-Bibliothek und Name
        fp = pcbnew.FootprintLoad(os.path.join(FOOTPRINT_DIR, lib + ".pretty"), name)  # laden
        if fp is None:                                        # nicht gefunden
            raise SystemExit(f"Footprint fehlt: {part['fp']}")  # abbrechen
        fp.SetFPID(pcbnew.LIB_ID(lib, name))                  # Bibliotheksverweis
        fp.SetReference(part["ref"])                          # Referenz
        fp.SetValue(part["value"])                            # Wert
        x, y, rot = part["pcb"]                               # Platzierung
        fp.SetPosition(pt(x, y))                              # Position
        fp.SetOrientationDegrees(rot)                         # Drehung
        fp.SetPath(pcbnew.KIID_PATH("/" + sch.symbol_uuids[part["ref"]]))  # Verknüpfung mit dem Schaltplan-Symbol
        board.Add(fp)                                         # zur Platine
        for pad in fp.Pads():                                 # jedes Pad
            net = nets.get((part["ref"], pad.GetNumber()))    # Netz aus der Netzliste
            if net in netinfo:                                # verbundenes Netz?
                pad.SetNet(netinfo[net])                      # zuweisen
        fp.Value().SetVisible(False)                          # Wert nicht drucken (Platz sparen)
        ref = fp.Reference()                                  # Referenztext
        ref.SetLayer(pcbnew.F_Fab)                            # Referenz in den Bestückungsplan (F.Fab), nicht auf die Platine
        ref.SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))    # klein
        ref.SetTextThickness(mm(0.12))                        # dünn
        if part["ref"] in D.TIGHT_HOLES:                      # Loch direkt neben einer Stiftleiste?
            for item in list(fp.GraphicalItems()):            # alle Grafiken des Footprints
                if item.GetLayer() in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):  # Sperrfläche (Courtyard)
                    fp.Remove(item)                           # entfernen (Abstand prüft die Bohrungsregel)
    draw_outline(board)                                       # Kontur und Fenster
    return board, netinfo                                     # Platine und Netze zurückgeben


def check_headers(board):                                     # prüft die Lage der UNO-Stiftleisten
    import pcbnew                                             # KiCad-Modul
    expect = {"J1": (27.94, 50.80, 45.72, 50.80), "J2": (50.80, 50.80, 63.50, 50.80),  # Pad 1 und letztes Pad
              "J3": (63.50, 2.54, 45.72, 2.54), "J4": (41.66, 2.54, 18.80, 2.54)}
    for fp in board.GetFootprints():                          # alle Footprints
        ref = fp.GetReference()                               # Referenz
        if ref not in expect:                                 # keine UNO-Stiftleiste
            continue                                          # überspringen
        pads = sorted(fp.Pads(), key=lambda p: int(p.GetNumber()))  # Pads nach Nummer
        first, last = pads[0].GetPosition(), pads[-1].GetPosition()  # erstes und letztes Pad
        got = (pcbnew.ToMM(first.x) - ORIGIN[0], pcbnew.ToMM(first.y) - ORIGIN[1],  # in Platinen-Koordinaten
               pcbnew.ToMM(last.x) - ORIGIN[0], pcbnew.ToMM(last.y) - ORIGIN[1])
        if any(abs(a - b) > 0.01 for a, b in zip(got, expect[ref])):  # Abweichung?
            raise SystemExit(f"{ref}: Pads liegen bei {got}, erwartet {expect[ref]}")  # abbrechen


def add_silkscreen(board):                                    # Beschriftungen auf dem Bestückungsdruck
    import pcbnew                                             # KiCad-Modul
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}  # Footprints nach Referenz
    for part in D.PARTS:                                      # jedes Bauteil
        if "labels" not in part:                              # keine Pin-Beschriftung gewünscht
            continue                                          # überspringen
        back = part["ref"] in D.BACK_LABELS                   # UNO-Leisten: Beschriftung auf der Unterseite
        fp = fps[part["ref"]]                                 # Footprint
        pads = sorted(fp.Pads(), key=lambda p: int(p.GetNumber()))  # Pads nach Nummer
        names = part["labels"].split()                        # Beschriftungen
        x0 = pcbnew.ToMM(pads[0].GetPosition().x) - ORIGIN[0]  # erstes Pad x
        x1 = pcbnew.ToMM(pads[-1].GetPosition().x) - ORIGIN[0]  # letztes Pad x
        horizontal = abs(x1 - x0) > 0.1                       # Pads waagerecht angeordnet?
        for pad, text in zip(pads, names):                    # jedes Pad
            px = pcbnew.ToMM(pad.GetPosition().x) - ORIGIN[0]  # Pad x
            py = pcbnew.ToMM(pad.GetPosition().y) - ORIGIN[1]  # Pad y
            if horizontal:                                    # Leiste waagerecht
                rotate = len(text) > 3                        # lange Namen senkrecht schreiben
                off = (0.85 + 0.35 + len(text) * 0.7 * 0.95 / 2) if rotate else 1.9  # Abstand vom Pad
                dy = off if py < 26.7 else -off               # oben → Text darunter, unten → darüber
                if back:                                      # Unterseite (gespiegelt)
                    t = add_text(board, text, px, py + dy, 0.7, layer=pcbnew.B_SilkS, angle=90 if rotate else 0)
                    t.SetMirrored(True)                       # von unten lesbar
                else:                                         # Oberseite
                    add_text(board, text, px, py + dy, 0.7, angle=90 if rotate else 0)  # Text
            else:                                             # Leiste senkrecht
                dx = 2.2 if px < 34 else -2.2                 # links → Text rechts, rechts → Text links
                add_text(board, text, px + dx, py, 0.7)       # Text
    for text, (x, y), angle in D.SILK_NAMES:                  # Namen der Steckverbinder
        add_text(board, text, x, y, 0.8, angle=angle, bold=True)  # Text
    jp2 = fps["JP2"].GetPosition()                            # Lage der Lötbrücke JP2
    add_text(board, "5V>VMOT", pcbnew.ToMM(jp2.x) - ORIGIN[0], pcbnew.ToMM(jp2.y) - ORIGIN[1] + 1.8, 0.7)  # Beschriftung
    t = add_text(board, "Watchmen Shield - Arduino UNO Q", 62.3, 30.0, 1.0, layer=pcbnew.B_SilkS, angle=90)  # Rückseite
    t.SetMirrored(True)                                       # gespiegelt (von unten lesbar)
    cx = (D.WINDOW[0] + D.WINDOW[2]) / 2                      # Mitte des Fensters
    add_text(board, "Fenster LED-Matrix UNO Q", cx, D.WINDOW[1] + 2.0, 0.9, layer=pcbnew.F_Fab)  # Hinweis auf Fab-Lage


# ------------------------------------------------------------------ Autorouting ------
def ses_import(board, ses_path, netinfo):                     # liest Freerouting-Ergebnis (SES) ein
    import pcbnew                                             # KiCad-Modul
    with open(ses_path, encoding="utf-8") as f:               # Datei öffnen
        root = sexpr_parse(f.read())[0]                       # parsen
    routes = find(root, "routes")                             # Abschnitt mit Leiterbahnen
    res = find(routes, "resolution")                          # Auflösung, z.B. (resolution um 10)
    unit_nm = {"um": 1000.0, "mm": 1e6, "mil": 25400.0, "inch": 2.54e7}[res[1]] / float(res[2])  # nm je Einheit
    layers = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu}       # Lagen
    vias = {}                                                 # Durchkontaktierungs-Typen
    for ps in find_all(find(routes, "library_out") or [], "padstack"):  # jede Via-Art
        m = re.search(r"_(\d+):(\d+)_um", unq(ps[1]))         # Durchmesser und Bohrung aus dem Namen
        vias[unq(ps[1])] = (int(m.group(1)) * 1000, int(m.group(2)) * 1000) if m else (600000, 300000)  # in nm
    count_tracks, count_vias = 0, 0                           # Zähler
    for net in find_all(find(routes, "network_out"), "net"):  # jedes Netz
        name = unq(net[1])                                    # Netzname
        ni = netinfo.get(name)                                # KiCad-Netz
        for wire in [w for w in net if isinstance(w, list) and w[0] == "wire"]:  # jede Leiterbahn
            path = find(wire, "path")                         # Polylinie
            layer = layers[unq(path[1])]                      # Lage
            width = int(float(path[2]) * unit_nm)             # Breite
            coords = [float(v) for v in path[3:] if not isinstance(v, list)]  # Koordinaten
            pts = [(int(coords[i] * unit_nm), int(-coords[i + 1] * unit_nm)) for i in range(0, len(coords), 2)]  # y gespiegelt
            for (x1, y1), (x2, y2) in zip(pts, pts[1:]):      # jedes Teilstück
                t = pcbnew.PCB_TRACK(board)                   # neue Bahn
                t.SetStart(pcbnew.VECTOR2I(x1, y1))           # Anfang
                t.SetEnd(pcbnew.VECTOR2I(x2, y2))             # Ende
                t.SetWidth(width)                             # Breite
                t.SetLayer(layer)                             # Lage
                if ni:                                        # Netz bekannt?
                    t.SetNet(ni)                              # zuweisen
                board.Add(t)                                  # zur Platine
                count_tracks += 1                             # zählen
        for via in [v for v in net if isinstance(v, list) and v[0] == "via"]:  # jede Durchkontaktierung
            dia, drill = vias.get(unq(via[1]), (600000, 300000))  # Größe
            v = pcbnew.PCB_VIA(board)                         # neue Via
            v.SetPosition(pcbnew.VECTOR2I(int(float(via[2]) * unit_nm), int(-float(via[3]) * unit_nm)))  # Position
            v.SetWidth(dia)                                   # Durchmesser
            v.SetDrill(drill)                                 # Bohrung
            v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)          # durchgehend
            if ni:                                            # Netz bekannt?
                v.SetNet(ni)                                  # zuweisen
            board.Add(v)                                      # zur Platine
            count_vias += 1                                   # zählen
    return count_tracks, count_vias                           # Anzahl zurückgeben


def fix_dsn_classes(dsn):                                     # ordnet die Netze in der DSN-Datei ihren Klassen zu
    with open(dsn, encoding="utf-8") as f:                    # Datei lesen
        text = f.read()                                       # Inhalt
    wanted = {}                                               # Netz → Klasse
    for name, nc in D.NETCLASSES.items():                     # jede Klasse
        for n in nc["nets"]:                                  # jedes Netz
            wanted[net_pattern(n)] = name                     # merken
    def repl(m):                                              # ersetzt einen (class ...)-Kopf
        cls = m.group(1)                                      # Klassenname
        nets = [unq(t) for t in TOKEN.findall(m.group(2))]    # Netze der Klasse
        if cls == "kicad_default":                            # Standardklasse
            nets = [n for n in nets if n not in wanted]       # zugeordnete Netze entfernen
        else:                                                 # eigene Klasse
            nets = [n for n in wanted if wanted[n] == cls]    # Netze dieser Klasse
        return f"(class {cls} " + " ".join(q(n) for n in nets) + "\n      (circuit"  # neuer Kopf
    text = re.sub(r'\(class ([^\s()]+)([^()]*)\(circuit', repl, text)  # alle Klassen bearbeiten (Netznamen ohne Klammern)
    text = re.sub(r'\(net "?GND"?\s*\(pins[^)]*\)\s*\)', '', text)  # Masse nicht routen (kommt über Kupferflächen)
    text = re.sub(r'(\(class [^\s()]+[^()]*?)\s"?GND"?(?=[\s)])', r'\1', text)  # Masse aus den Klassenlisten
    with open(dsn, "w", encoding="utf-8") as f:               # Datei schreiben
        f.write(text)                                         # speichern


def autoroute(board, pcb_path, netinfo, jar, java, passes):   # Autorouting mit Freerouting
    import pcbnew                                             # KiCad-Modul
    dsn = pcb_path.replace(".kicad_pcb", ".dsn")              # Specctra-Entwurf
    ses = pcb_path.replace(".kicad_pcb", ".ses")              # Specctra-Sitzung (Ergebnis)
    if not pcbnew.ExportSpecctraDSN(board, dsn):              # Entwurf exportieren
        raise SystemExit("DSN-Export fehlgeschlagen")         # abbrechen
    fix_dsn_classes(dsn)                                      # Netze den richtigen Klassen zuordnen
    cmd = [java, "-jar", jar, "-de", dsn, "-do", ses, "-mp", str(passes), "--gui.enabled=false"]  # Freerouting
    print("  Freerouting läuft ...", flush=True)               # Fortschritt
    subprocess.run(cmd, capture_output=True, text=True, timeout=1800)  # ausführen (max. 30 min)
    if not os.path.exists(ses):                               # kein Ergebnis
        raise SystemExit("Freerouting hat keine SES-Datei erzeugt")  # abbrechen
    tracks, vias = ses_import(board, ses, netinfo)            # Ergebnis übernehmen
    print(f"  {tracks} Bahnstücke, {vias} Durchkontaktierungen übernommen")  # Info
    for p in (dsn, ses):                                      # Zwischendateien
        os.replace(p, os.path.join(OUT, "routing", os.path.basename(p)))  # in Unterordner verschieben


def add_edge_keepouts(board, width=0.8):                      # Sperrflächen für Leiterbahnen entlang aller Kanten
    import math                                               # Winkelrechnung
    import pcbnew                                             # KiCad-Modul
    x1, y1, x2, y2 = D.WINDOW                                 # Fenster
    window = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]         # Fenster als Rechteck (Ecken großzügig)
    for poly in (D.OUTLINE, window):                          # Außenkontur und Fenster
        pts = poly + [poly[0]]                                # schließen
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):          # jede Kante
            length = math.hypot(bx - ax, by - ay)             # Länge
            nx, ny = -(by - ay) / length * width / 2, (bx - ax) / length * width / 2  # Normale (halbe Breite)
            ex, ey = (bx - ax) / length * width / 2, (by - ay) / length * width / 2  # Verlängerung an den Enden
            quad = [(ax - ex + nx, ay - ey + ny), (bx + ex + nx, by + ey + ny),  # Streifen um die Kante
                    (bx + ex - nx, by + ey - ny), (ax - ex - nx, ay - ey - ny)]
            z = pcbnew.ZONE(board)                            # neue Regelfläche
            z.SetIsRuleArea(True)                             # Sperrfläche
            z.SetDoNotAllowTracks(True)                       # keine Leiterbahnen
            z.SetDoNotAllowVias(True)                         # keine Durchkontaktierungen
            z.SetDoNotAllowPads(False)                        # Pads erlaubt (Stiftleisten am Rand)
            z.SetDoNotAllowFootprints(False)                  # Bauteile erlaubt
            z.SetDoNotAllowCopperPour(False)                  # Masseflächen erlaubt (halten eigenen Randabstand)
            ls = pcbnew.LSET()                                # Lagen
            ls.AddLayer(pcbnew.F_Cu)                          # oben
            ls.AddLayer(pcbnew.B_Cu)                          # unten
            z.SetLayerSet(ls)                                 # zuweisen
            outline = z.Outline()                             # Umriss
            outline.NewOutline()                              # neue Kontur
            for x, y in quad:                                 # Eckpunkte
                outline.Append(mm(ORIGIN[0] + x), mm(ORIGIN[1] + y))  # anhängen
            board.Add(z)                                      # zur Platine


def fix_starved_thermals(board, report_text):                 # Pads mit zu wenig Wärmefallen-Stegen voll anbinden
    import pcbnew                                             # KiCad-Modul
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}  # Footprints nach Referenz
    fixed = 0                                                 # Anzahl geänderter Pads
    for num, ref in re.findall(r"\[starved_thermal\].*?pad (\S+) \[[^\]]*\] of (\S+)", report_text, re.S | re.I):  # betroffene Pads
        for pad in fps[ref].Pads():                           # Pads des Bauteils
            if pad.GetNumber() == num:                        # passendes Pad
                pad.SetZoneConnection(pcbnew.ZONE_CONNECTION_FULL)  # voll mit der Fläche verbinden
                fixed += 1                                    # zählen
    if fixed:                                                 # etwas geändert?
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())         # Flächen neu füllen
    return fixed                                              # Anzahl zurückgeben


def point_in_poly(x, y, poly):                                # liegt ein Punkt in einem Polygon? (Strahl-Methode)
    inside = False                                            # Ergebnis
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):  # jede Kante
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):  # Strahl schneidet die Kante
            inside = not inside                               # umschalten
    return inside                                             # zurückgeben


def free_spot(board, x, y, radius, gnd_code, skip=()):        # ist an (x, y) Platz für Kupfer des GND-Netzes?
    import pcbnew                                             # KiCad-Modul
    edge = 0.5 + radius                                       # Mindestabstand zum Rand (außerhalb der Rand-Sperrstreifen)
    if not point_in_poly(x, y, D.OUTLINE):                    # außerhalb der Platine
        return False                                          # nein
    wx1, wy1, wx2, wy2 = D.WINDOW                             # Fenster
    if wx1 - edge < x < wx2 + edge and wy1 - edge < y < wy2 + edge:  # im/zu nah am Fenster
        return False                                          # nein
    pts = D.OUTLINE + [D.OUTLINE[0]]                          # Außenkontur
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):              # Abstand zu jeder Außenkante
        dx, dy = bx - ax, by - ay                             # Kantenrichtung
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))  # nächster Punkt
        if ((ax + t * dx - x) ** 2 + (ay + t * dy - y) ** 2) ** 0.5 < edge:  # zu nah
            return False                                      # nein
    pos = pt(x, y)                                            # Punkt in KiCad-Einheiten
    acc = mm(radius + 0.25)                                   # Abstand inkl. Sicherheitsreserve
    for fp in board.GetFootprints():                          # alle Pads
        for pad in fp.Pads():                                 # jedes Pad
            if pad in skip:                                   # ausdrücklich ausgenommen (Pad, das angebunden wird)
                continue                                      # überspringen
            if pad.GetNetCode() == gnd_code and gnd_code != 0:  # Masse-Pad: berühren erlaubt, aber nicht im Pad/zu nah an der Bohrung
                extra = mm(radius + 0.3) if pad.GetDrillSize().x > 0 else mm(radius + 0.05)  # Mindestabstand
                if pad.HitTest(pos, extra):                   # zu nah
                    return False                              # nein
                continue                                      # sonst in Ordnung
            if pad.HitTest(pos, acc + (mm(0.3) if pad.GetDrillSize().x > 0 else 0)):  # fremdes Netz: voller Abstand
                return False                                  # nein
    for t in board.GetTracks():                               # Bahnen und Vias
        if t in skip:                                         # ausdrücklich ausgenommen
            continue                                          # überspringen
        if t.GetClass() == "PCB_VIA":                         # Via (auch eigene Masse-Vias: Bohrungsabstand!)
            if t.HitTest(pos, mm(radius + 0.45)):             # Mindestabstand zwischen Bohrungen
                return False                                  # nein
            continue                                          # weiter
        if t.GetNetCode() == gnd_code:                        # eigene Masse-Bahn ist erlaubt
            continue                                          # überspringen
        if t.HitTest(pos, acc):                               # zu nah
            return False                                      # nein
    return True                                               # Platz ist frei


def add_gnd_via(board, gnd, x, y, pad=None):                  # setzt eine Masse-Via (optional mit Bahn zum Pad)
    import pcbnew                                             # KiCad-Modul
    v = pcbnew.PCB_VIA(board)                                 # neue Via
    v.SetPosition(pt(x, y))                                   # Position
    v.SetWidth(mm(0.6))                                       # Durchmesser
    v.SetDrill(mm(0.3))                                       # Bohrung
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)                  # durchgehend
    v.SetNet(gnd)                                             # Masse
    board.Add(v)                                              # zur Platine
    if pad is not None:                                       # Verbindung zum Pad
        t = pcbnew.PCB_TRACK(board)                           # neue Bahn
        t.SetStart(pad.GetPosition())                         # Anfang am Pad
        t.SetEnd(v.GetPosition())                             # Ende an der Via
        t.SetWidth(mm(0.4))                                   # Breite
        t.SetLayer(pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu)  # Lage des Pads
        t.SetNet(gnd)                                         # Masse
        board.Add(t)                                          # zur Platine
    return v                                                  # Via zurückgeben


def connect_gnd_pads(board, netinfo):                         # bindet jedes Masse-Pad mit einer nahen Via an
    import math                                               # Winkel
    import pcbnew                                             # KiCad-Modul
    gnd = netinfo["GND"]                                      # Masse-Netz
    code = gnd.GetNetCode()                                   # Netznummer
    added = 0                                                 # Zähler
    for fp in board.GetFootprints():                          # alle Bauteile
        for pad in fp.Pads():                                 # alle Pads
            if pad.GetNetCode() != code or pad.GetDrillSize().x > 0:  # nur SMD-Masse-Pads (THT liegt auf beiden Lagen)
                continue                                      # überspringen
            px = pcbnew.ToMM(pad.GetPosition().x) - ORIGIN[0]  # Pad x
            py = pcbnew.ToMM(pad.GetPosition().y) - ORIGIN[1]  # Pad y
            done = False                                      # schon angebunden?
            for dist in (1.2, 1.5, 1.9, 2.4):                 # Abstände probieren
                for k in range(16):                           # 16 Richtungen
                    a = 2 * math.pi * k / 16                  # Winkel
                    x, y = px + dist * math.cos(a), py + dist * math.sin(a)  # Kandidat
                    if not free_spot(board, x, y, 0.3, code):  # Via passt nicht
                        continue                              # weiter
                    ok = all(free_spot(board, px + (x - px) * f, py + (y - py) * f, 0.2, code, skip=(pad,))  # Bahn frei?
                             for f in (0.45, 0.6, 0.75, 0.9))
                    if ok:                                    # alles frei
                        add_gnd_via(board, gnd, x, y, pad)    # Via + Bahn setzen
                        added += 1                            # zählen
                        done = True                           # fertig
                        break                                 # nächste Richtung nicht nötig
                if done:                                      # angebunden
                    break                                     # nächstes Pad
    return added                                              # Anzahl zurückgeben


def stitch_vias(board, netinfo, step=3.0):                    # verteilte Masse-Vias verbinden obere und untere Fläche
    gnd = netinfo["GND"]                                      # Masse-Netz
    code = gnd.GetNetCode()                                   # Netznummer
    added = 0                                                 # Zähler
    y = 1.5                                                   # Startzeile
    while y < 53.0:                                           # über die ganze Höhe
        x = 1.5                                               # Startspalte
        while x < 68.0:                                       # über die ganze Breite
            if free_spot(board, x, y, 0.3 + 0.35, code):      # großzügig frei?
                add_gnd_via(board, gnd, x, y)                 # Via setzen
                added += 1                                    # zählen
            x += step                                         # nächste Spalte
        y += step                                             # nächste Zeile
    return added                                              # Anzahl zurückgeben


def remove_dangling_vias(board, report_text):                 # entfernt Vias, die laut DRC nirgends anschließen
    import pcbnew                                             # KiCad-Modul
    pos = set()                                               # Positionen der hängenden Vias
    for x, y in re.findall(r"\[via_dangling\][^@]*@\(([-\d.]+) mm, ([-\d.]+) mm\)", report_text):  # aus dem Bericht
        pos.add((round(float(x), 3), round(float(y), 3)))     # merken
    removed = 0                                               # Zähler
    for t in list(board.GetTracks()):                         # alle Bahnen/Vias
        if t.GetClass() == "PCB_VIA":                         # nur Vias
            p = (round(pcbnew.ToMM(t.GetPosition().x), 3), round(pcbnew.ToMM(t.GetPosition().y), 3))  # Position
            if p in pos:                                      # hängt?
                board.Remove(t)                               # entfernen
                removed += 1                                  # zählen
    return removed                                            # Anzahl zurückgeben


def stitch_islands(board, netinfo):                           # setzt Vias in Masse-Inseln ohne Verbindung zur anderen Lage
    import pcbnew                                             # KiCad-Modul
    gnd = netinfo["GND"]                                      # Masse-Netz
    code = gnd.GetNetCode()                                   # Netznummer
    anchors = [t.GetPosition() for t in board.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetNetCode() == code]  # Masse-Vias
    anchors += [p.GetPosition() for fp in board.GetFootprints() for p in fp.Pads()  # Masse-Pads mit Bohrung
                if p.GetNetCode() == code and p.GetDrillSize().x > 0]
    added = 0                                                 # Zähler
    for z in board.Zones():                                   # alle Flächen
        if z.GetIsRuleArea() or z.GetNetCode() != code:       # nur Masseflächen
            continue                                          # überspringen
        polys = z.GetFilledPolysList(z.GetLayer())            # gefüllte Inseln
        for i in range(polys.OutlineCount()):                 # jede Insel
            chain = polys.Outline(i)                          # Umriss
            if any(chain.PointInside(a) for a in anchors):    # hat schon eine Verbindung zur anderen Lage
                continue                                      # in Ordnung
            box = chain.BBox()                                # umschließendes Rechteck
            x0, y0 = pcbnew.ToMM(box.GetX()) - ORIGIN[0], pcbnew.ToMM(box.GetY()) - ORIGIN[1]  # links oben
            w, h = pcbnew.ToMM(box.GetWidth()), pcbnew.ToMM(box.GetHeight())  # Größe
            placed = False                                    # schon gesetzt?
            steps = [k * 0.4 for k in range(int(max(w, h) / 0.4) + 1)]  # Raster 0,4 mm
            for dy in steps:                                  # Zeilen
                for dx in steps:                              # Spalten
                    if dx > w or dy > h:                      # außerhalb des Rechtecks
                        continue                              # überspringen
                    x, y = x0 + dx, y0 + dy                   # Kandidat
                    if chain.PointInside(pt(x, y)) and free_spot(board, x, y, 0.3, code):  # in der Insel und frei
                        anchors.append(add_gnd_via(board, gnd, x, y).GetPosition())  # Via setzen
                        added += 1                            # zählen
                        placed = True                         # fertig
                        break                                 # Spalten-Schleife verlassen
                if placed:                                    # gesetzt
                    break                                     # Zeilen-Schleife verlassen
    if added:                                                 # etwas geändert?
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())         # neu füllen
    return added                                              # Anzahl zurückgeben


def add_ground_planes(board, netinfo):                        # Masseflächen oben und unten
    import pcbnew                                             # KiCad-Modul
    gnd = netinfo["GND"]                                      # Masse-Netz
    poly = [(-1.0, -1.0), (70.0, -1.0), (70.0, 55.0), (-1.0, 55.0)]  # größer als die Platine (Kontur begrenzt)
    zone(board, gnd, pcbnew.F_Cu, poly)                       # Fläche oben
    zone(board, gnd, pcbnew.B_Cu, poly)                       # Fläche unten
    filler = pcbnew.ZONE_FILLER(board)                        # Füll-Werkzeug
    filler.Fill(board.Zones())                                # Flächen füllen (Regelflächen werden übersprungen)


def drc(board, report_path):                                  # Design-Rule-Check
    import pcbnew                                             # KiCad-Modul
    pcbnew.WriteDRCReport(board, report_path, pcbnew.EDA_UNITS_MILLIMETRES, True)  # Bericht schreiben
    with open(report_path, encoding="utf-8") as f:            # Bericht lesen
        text = f.read()                                       # Inhalt
    kinds = re.findall(r"^\[(\w+)\]", text, re.M)           # Art jedes Eintrags
    counts = defaultdict(int)                                 # Anzahl je Art
    for k in kinds:                                           # zählen
        counts[k] += 1                                        # +1
    relevant = {k: v for k, v in counts.items() if k not in IGNORED_DRC and k != "unconnected_items"}  # echte Probleme
    unconn = counts.get("unconnected_items", 0)               # offene Verbindungen
    return relevant, unconn, text                             # zurückgeben


# ------------------------------------------------------------------ Ausgaben ---------
def write_bom(path):                                          # Stückliste (gruppiert)
    groups = defaultdict(list)                                # (Wert, Footprint) → Referenzen
    for part in D.PARTS:                                      # jedes Bauteil
        groups[(part["value"], part["fp"], part["sym"])].append(part["ref"])  # gruppieren
    with open(path, "w", newline="", encoding="utf-8") as f:  # Datei öffnen
        w = csv.writer(f, delimiter=";")                      # Semikolon (Excel, deutsch)
        w.writerow(["Anzahl", "Referenzen", "Wert", "Footprint", "Symbol"])  # Kopfzeile
        for (value, fp, sym), refs in sorted(groups.items(), key=lambda g: g[1][0]):  # sortiert
            w.writerow([len(refs), ",".join(refs), value, fp, sym])  # Zeile


def svg_to_png(docs):                                         # PNG-Vorschaubilder (wenn cairosvg installiert ist)
    try:                                                      # optional
        import cairosvg                                       # SVG-Umwandlung
    except ImportError:                                       # nicht vorhanden
        print("   (cairosvg fehlt – keine PNG-Vorschau erzeugt)")  # Hinweis
        return                                                # fertig
    for name in ("platine_oben", "platine_unten", D.BOARD_NAME):  # Bilder
        src = os.path.join(docs, name + ".svg")               # Quelle
        dst = os.path.join(docs, ("schaltplan" if name == D.BOARD_NAME else name) + ".png")  # Ziel
        cairosvg.svg2png(url=src, write_to=dst, output_width=2400 if name == D.BOARD_NAME else 1600,
                         background_color="white")            # umwandeln


def export_outputs(pcb_path, sch_path):                       # Fertigungsdaten und Doku
    fab = os.path.join(OUT, "fabrication")                    # Fertigungsordner
    gerber = os.path.join(fab, "gerber")                      # Gerber-Ordner
    docs = os.path.join(OUT, "docs")                          # Doku-Ordner
    for d in (fab, gerber, docs):                             # Ordner anlegen
        os.makedirs(d, exist_ok=True)                         # falls nicht vorhanden
    run(["kicad-cli", "pcb", "export", "gerbers", "--output", gerber + "/",  # Gerber-Dateien
         "--layers", "F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts", pcb_path])
    run(["kicad-cli", "pcb", "export", "drill", "--output", gerber + "/", "--format", "excellon",  # Bohrdaten
         "--excellon-units", "mm", "--generate-map", "--map-format", "pdf", pcb_path])
    shutil.make_archive(os.path.join(fab, D.BOARD_NAME + "-gerber"), "zip", gerber)  # ZIP für den Hersteller
    run(["kicad-cli", "pcb", "export", "pos", "--output", os.path.join(fab, D.BOARD_NAME + "-bestueckung.csv"),  # Bestückung
         "--format", "csv", "--units", "mm", "--side", "both", pcb_path])
    write_bom(os.path.join(fab, D.BOARD_NAME + "-stueckliste.csv"))  # Stückliste
    run(["kicad-cli", "sch", "export", "pdf", "--output", os.path.join(docs, "schaltplan.pdf"), sch_path])  # Schaltplan-PDF
    run(["kicad-cli", "sch", "export", "svg", "--output", docs + "/", "--no-background-color", sch_path])  # Schaltplan-SVG
    run(["kicad-cli", "pcb", "export", "pdf", "--output", os.path.join(docs, "outline_1zu1.pdf"),  # 1:1-Prüfvorlage
         "--layers", "Edge.Cuts,F.SilkS", "--black-and-white", pcb_path])
    run(["kicad-cli", "pcb", "export", "pdf", "--output", os.path.join(docs, "bestueckungsplan.pdf"),  # Bestückungsplan
         "--layers", "Edge.Cuts,F.Fab,F.SilkS", "--black-and-white", pcb_path])
    for name, layers in (("platine_oben.svg", "F.Cu,F.SilkS,F.Mask,Edge.Cuts"),  # Ansicht oben
                         ("platine_unten.svg", "B.Cu,B.SilkS,Edge.Cuts")):  # Ansicht unten
        run(["kicad-cli", "pcb", "export", "svg", "--output", os.path.join(docs, name), "--layers", layers,
             "--page-size-mode", "2", "--exclude-drawing-sheet", pcb_path])
    svg_to_png(docs)                                          # Vorschaubilder


# ------------------------------------------------------------------ Hauptprogramm ----
def main():
    ap = argparse.ArgumentParser(description="Watchmen-Shield erzeugen")  # Parameter
    ap.add_argument("--freerouting", default=os.environ.get("FREEROUTING_JAR", ""), help="Pfad zu freerouting.jar")
    ap.add_argument("--java", default=os.environ.get("JAVA", "java"), help="Java 25 (für Freerouting 2.4)")
    ap.add_argument("--passes", type=int, default=40, help="max. Durchläufe des Autorouters")
    ap.add_argument("--no-route", action="store_true", help="ohne Autorouting (nur Platzierung)")
    args = ap.parse_args()                                    # auswerten
    import pcbnew                                             # KiCad-Modul (bricht hier ab, wenn KiCad fehlt)
    os.makedirs(os.path.join(OUT, "routing"), exist_ok=True)  # Ausgabeordner anlegen
    sch_path = os.path.join(OUT, D.BOARD_NAME + ".kicad_sch")  # Schaltplan
    pcb_path = os.path.join(OUT, D.BOARD_NAME + ".kicad_pcb")  # Platine
    pro_path = os.path.join(OUT, D.BOARD_NAME + ".kicad_pro")  # Projekt

    print("1) Schaltplan", flush=True)                        # Fortschritt
    sch = build_schematic(sch_path)                           # Schaltplan schreiben
    print("2) Netzliste prüfen", flush=True)                  # Fortschritt
    nets, members = read_netlist(sch_path, os.path.join(OUT, "routing", D.BOARD_NAME + ".net"))  # Netzliste
    errors = verify_netlist(nets, members)                    # vergleichen
    if errors:                                                # Abweichungen?
        print("\n".join(errors))                              # anzeigen
        raise SystemExit("Netzliste weicht vom Entwurf ab")   # abbrechen
    print(f"   {len([m for m in members if not m.startswith('unconnected')])} Netze ok", flush=True)  # Info

    print("3) Platine", flush=True)                           # Fortschritt
    pcbnew.SaveBoard(pcb_path, pcbnew.BOARD())                # leere Platine (und Projektdatei) anlegen
    board, netinfo = build_pcb(pcb_path, sch, nets)           # Platine aufbauen
    check_headers(board)                                      # Stiftleisten-Lage prüfen
    add_silkscreen(board)                                     # Beschriftung
    add_edge_keepouts(board)                                  # Randabstand für den Autorouter
    if not args.no_route:                                     # Autorouting gewünscht?
        print("4) Autorouting", flush=True)                   # Fortschritt
        if not args.freerouting or not os.path.exists(args.freerouting):  # Freerouting fehlt
            raise SystemExit("freerouting.jar nicht gefunden (--freerouting)")  # abbrechen
        autoroute(board, pcb_path, netinfo, args.freerouting, args.java, args.passes)  # routen
    print(f"   Masse-Vias an SMD-Pads: {connect_gnd_pads(board, netinfo)}", flush=True)  # Masse-Pads anbinden
    print(f"   Verteilte Masse-Vias: {stitch_vias(board, netinfo)}", flush=True)  # Flächen verbinden
    add_ground_planes(board, netinfo)                         # Masseflächen
    for _ in range(3):                                        # bis zu 3 Runden
        n = stitch_islands(board, netinfo)                    # Inseln ohne Verbindung anbinden
        print(f"   Insel-Vias: {n}", flush=True)              # Info
        if n == 0:                                            # nichts mehr zu tun
            break                                             # fertig
    pcbnew.SaveBoard(pcb_path, board)                         # speichern (schreibt auch .kicad_pro)
    patch_project(pro_path)                                   # Netzklassen-Zuordnung für die KiCad-Oberfläche
    os.makedirs(os.path.join(OUT, "docs"), exist_ok=True)     # Doku-Ordner
    relevant, unconn, report = drc(board, os.path.join(OUT, "docs", "drc_bericht.txt"))  # Regelprüfung
    if remove_dangling_vias(board, report):                   # Vias ohne Anschluss?
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())         # Flächen neu füllen
        relevant, unconn, report = drc(board, os.path.join(OUT, "docs", "drc_bericht.txt"))  # erneut prüfen
    if fix_starved_thermals(board, report):                   # schwach angebundene Masse-Pads?
        pcbnew.SaveBoard(pcb_path, board)                     # speichern
        patch_project(pro_path)                               # Projektdatei wieder ergänzen
        relevant, unconn, report = drc(board, os.path.join(OUT, "docs", "drc_bericht.txt"))  # erneut prüfen
    print(f"   DRC: {dict(relevant) or 'keine Verstöße'}, {unconn} offene Verbindungen", flush=True)  # Ergebnis
    print("5) Fertigungsdaten", flush=True)                   # Fortschritt
    export_outputs(pcb_path, sch_path)                        # Gerber, PDF, SVG ...
    print(f"Fertig: {OUT}")                                   # Abschluss
    return 0 if (unconn == 0 and not relevant) else 1         # Fehlercode bei Problemen


if __name__ == "__main__":
    sys.exit(main())                                          # starten

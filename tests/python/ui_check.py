"""
Browser-Test der Weboberfläche gegen den Simulations-Server (Playwright + Chromium).
Spielt die Kalibrierung über die Webseite durch und speichert Bildschirmfotos.

Aufruf:  python3 tests/python/ui_check.py [Ausgabeordner]
"""
import os
import subprocess
import sys
import time

from playwright.sync_api import sync_playwright

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "screenshots")
PORT = 7071
FAILS = []


def check(cond, text):
    print(("  OK   " if cond else "  FAIL ") + text, flush=True)
    if not cond:
        FAILS.append(text)


def main():
    os.makedirs(OUT, exist_ok=True)
    server = subprocess.Popen([sys.executable, os.path.join(HERE, "sim_server.py"), "--port", str(PORT)],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(3)
        with sync_playwright() as p:
            exe = "/opt/pw-browsers/chromium" if os.path.exists("/opt/pw-browsers/chromium") else None
            browser = p.chromium.launch(executable_path=exe if exe and os.path.isfile(exe) else None)
            page = browser.new_page(viewport={"width": 1400, "height": 1000})
            errors = []
            page.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(f"http://127.0.0.1:{PORT}/")
            page.wait_for_function("document.getElementById('stateBadge').textContent.includes('Nicht kalibriert')", timeout=15000)
            check(True, "Seite lädt, Zustand 'Nicht kalibriert'")
            time.sleep(1.5)
            page.screenshot(path=os.path.join(OUT, "1_live_unkalibriert.png"))

            page.click("button[data-tab=calib]")
            page.wait_for_selector("#calibBody:not(.hidden)", timeout=5000)
            check(True, "Kalibrier-Reiter zeigt Formular")
            page.fill("#tiltSlider", "110")
            page.dispatch_event("#tiltSlider", "input")
            page.wait_for_function("document.getElementById('kvTilt').textContent.startsWith('110')", timeout=5000)
            check(True, "Schieberegler bewegt Servo 1 auf 110°")
            page.click("button[data-take=tilt_max]")
            check(page.input_value("#tilt_max") == "110.0", "'Aktuell → Maximum' übernimmt 110°")
            page.fill("#tilt_max", "135")
            page.fill("#tiltSlider", "90")
            page.dispatch_event("#tiltSlider", "input")
            page.click("button[data-cmd=cal_home]")
            page.wait_for_function("document.getElementById('calHomed').textContent.includes('bekannt') && !document.getElementById('calHomed').textContent.includes('unbekannt')", timeout=8000)
            check(True, "Referenzfahrt über die Webseite")
            page.click("button[data-jog='500']")
            page.click("button[data-jog='500']")
            page.wait_for_function("parseInt(document.getElementById('calPan').textContent) >= 1000", timeout=5000)
            check(True, "Joggen der Drehachse")
            page.click("button[data-take=pan_center]")
            page.fill("#pan_center", "1500")
            time.sleep(0.5)
            page.screenshot(path=os.path.join(OUT, "2_kalibrierung1.png"), full_page=True)
            page.click("#btnSave1")
            page.wait_for_function("document.getElementById('step1').classList.contains('done')", timeout=5000)
            check(True, "Kalibrierung 1 gespeichert (Schritt 1 erledigt)")
            check(not page.is_disabled("#btnStart2"), "'Bereit zur Kamerabewegung einstellen' freigeschaltet")
            page.click("#btnStart2")
            page.wait_for_function("document.getElementById('stateBadge').textContent.includes('Kalibrierung 2')", timeout=5000)
            time.sleep(1.0)
            page.screenshot(path=os.path.join(OUT, "3_kalibrierung2_laeuft.png"), full_page=True)
            page.wait_for_function("document.getElementById('stateBadge').textContent.includes('Kalibrierung abgeschlossen')", timeout=40000)
            check(True, "Kalibrierung 2 automatisch abgeschlossen")
            page.screenshot(path=os.path.join(OUT, "4_kalibriert.png"), full_page=True)
            page.wait_for_function("document.getElementById('stateBadge').textContent.includes('Objekterkennung aktiv')", timeout=10000)
            check(True, "Objekterkennung aktiv")

            page.click("button[data-tab=live]")
            page.wait_for_function("document.getElementById('labelBox').textContent.includes('HUMEN')", timeout=15000)
            check(True, "Live: 'HUMEN' angezeigt")
            page.wait_for_function("document.querySelectorAll('#radarTable tr').length >= 3", timeout=5000)
            check(True, "Radar-Tabelle zeigt 2 Ziele")
            time.sleep(3)
            page.screenshot(path=os.path.join(OUT, "5_live_aktiv.png"), full_page=True)

            page.click("button[data-tab=settings]")
            time.sleep(0.5)
            check(page.input_value("input[name=sleep_timeout_s]") == "120", "Einstellungen zeigen gespeicherte Werte")
            page.fill("input[name=vibration_threshold_mg]", "250")
            page.click("#settingsForm button[type=submit]")
            time.sleep(1.0)
            page.reload()
            page.click("button[data-tab=settings]")
            page.wait_for_function("document.querySelector('input[name=vibration_threshold_mg]').value === '250'", timeout=5000)
            check(True, "Einstellung gespeichert und nach Neuladen vorhanden")
            page.screenshot(path=os.path.join(OUT, "6_einstellungen.png"), full_page=True)

            page.click("button[data-tab=marker]")
            page.wait_for_function("document.getElementById('markerImg').complete && document.getElementById('markerImg').naturalWidth > 0", timeout=5000)
            check(True, "Marker wird angezeigt")
            page.screenshot(path=os.path.join(OUT, "7_muster.png"))

            page.click("button[data-tab=live]")
            page.click("button[data-cmd=sleep]")
            page.wait_for_function("document.getElementById('stateBadge').textContent.includes('Ruhemodus')", timeout=5000)
            time.sleep(1.5)
            page.screenshot(path=os.path.join(OUT, "8_ruhemodus.png"))
            check(True, "Ruhemodus über die Webseite")
            page.click("button[data-cmd=wake]")
            page.wait_for_function("document.getElementById('stateBadge').textContent.includes('Objekterkennung aktiv')", timeout=5000)
            check(True, "Aufwecken über die Webseite")

            page.set_viewport_size({"width": 390, "height": 844})
            time.sleep(1)
            page.screenshot(path=os.path.join(OUT, "9_handy.png"), full_page=True)
            check(not errors, "keine JavaScript-Fehler" + (": " + "; ".join(errors[:3]) if errors else ""))
            browser.close()
    finally:
        server.terminate()
        try:
            out, _ = server.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()
            out = ""
        bad = [ln for ln in (out or "").splitlines() if "Traceback" in ln or "ERROR" in ln or "Exception" in ln]
        check(not bad, "Server ohne Fehler" + (": " + bad[0] if bad else ""))
    print("ALLE UI-TESTS BESTANDEN" if not FAILS else f"{len(FAILS)} FEHLER")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()

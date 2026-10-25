# Projektlog: ESP32-S3 IR Whiteboard

Stand 2026-10-25. Dieses Log hält die wichtigsten Entscheidungen fest, damit die Arbeit später fortgesetzt werden kann.

## Zielhardware und Build

- Sensor: DFRobot SEN0158 IR-Positioning-Modul, liest bis zu vier IR-Blobs per I2C; kein Kamerabild.
- Offizielle Referenzen: https://wiki.dfrobot.com/sen0158/docs/20468 (Arduino-Init, Pointer `0x36`, 16-Byte-Read/Offsets), https://wiki.dfrobot.com/sen0158 (3.3/5 V, 0-3 m, Sichtwinkel/Pinout), https://github.com/DFRobot/I2C_Addresses (7-Bit-Adresse `0x58`).
- Zielboard: Waveshare ESP32-S3-Zero, 4 MB Flash, native USB-OTG für HID.
- SDA GPIO 5, SCL GPIO 6, I2C-Adresse `0x58`, Bus 100 kHz.
- PlatformIO-Profil: `waveshare_s3_zero_4mb`, Plattform Espressif32 6.9.0, Arduino.
- Build: `bash scripts/build-release.sh`; bettet `data/index.html` in die Firmware ein und hinterlässt ausschließlich vier gemeinsam zeitgestempelte Binärdateien im Projekt-Root. Statische Aliasdateien und ältere Zeitstempelsätze werden entfernt.
- Flash-Adressen: Bootloader `0x0000`, Partitionstabelle `0x8000`, boot_app0 `0xE000`, Firmware `0x10000`. Kein separates Dateisystem-Image.

## Implementiert

- USB-HID-Touchscreen als Standardmodus und optionaler USB-Mausmodus.
- IR-Punkt aktiv hält Touch; IR-Punktverlust gibt Touch frei. Mausmodus bietet Klick, Drag-and-drop und einstellbaren Long-Press-Rechtsklick.
- Vier-Punkt-Homographie. Quell- und Zielpunkte, HID-Modus, Auflösung, Glättung, Blob-Mindestgröße, Empfindlichkeit, WLAN-Zugangsdaten und Long-Press-Zeit liegen im NVS.
- Kalibrierung läuft im Browser-Vollbild, pausiert HID und erfasst jede Ecke automatisch nach 2,2 Sekunden stabiler Erkennung. Zwischen Ecken wird eine deutliche Bewegung verlangt. Quell- und Zielmarker sind in den zwei Live-Canvases verschiebbar; beim Loslassen wird die Homographie neu berechnet und persistiert.
- AP `IR-Whiteboard` mit Passwort `whiteboard` bleibt im AP+STA-Betrieb verfügbar; Web-UI unter `http://192.168.4.1`.
- AP wird vor I2C/Sensorinitialisierung gestartet, Startfehler geprüft und es gibt einen AP-only-Fallback. Status liefert `apStarted`.
- Weboberfläche wird zur Buildzeit als PROGMEM-Header in die Firmware eingebettet; LittleFS ist nicht erforderlich.
- Liveansicht hat zwei Koordinatenräume: Sensorfeld mit IR-Blobs und Kalibrier-Viereck; Bildschirm-Rechteck mit Zielpunkten und transformierter HID-Position.
- Kein Web-Flasher im Repository. Nur die fünf aktuellen Timestamp-Binärdateien und deren Flash-Offsets werden benötigt.
- Initialisierung und I2C-Read entsprechen nun der offiziellen DFRobot-Beispielreihenfolge; Empfindlichkeitsblöcke bleiben projektspezifische Presets aus der alten Firmware und sind nicht durch DFRobot-Beispielcode bestätigt.

## Noch zu prüfen

- Firmware wurde mit PlatformIO kompiliert; reale USB-Erkennung und Touch-/Mausverhalten konnten mangels angeschlossener Hardware nicht getestet werden.
- Insbesondere HID-Touchscreen-Erkennung unter Windows/Android sowie Rechtsklick-/Drag-Gesten am konkreten Host prüfen.
- Sensorreichweite hängt von IR-LED, Optik, Ausrichtung und Umgebungslicht ab; eine maximale Distanz ist nicht verifiziert.
- Das AP-Passwort ist aktuell fest im Firmwarecode und sollte vor gemeinsam genutztem Einsatz änderbar gemacht werden.
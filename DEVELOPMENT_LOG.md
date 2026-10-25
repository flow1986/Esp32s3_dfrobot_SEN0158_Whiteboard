# Projektlog: ESP32-S3 IR Whiteboard

Stand 2026-10-25. Dieses Log hält die wichtigsten Entscheidungen fest, damit die Arbeit später fortgesetzt werden kann.

## Zielhardware und Build

- Sensor: DFRobot SEN0158 IR-Positioning-Modul, liest bis zu vier IR-Blobs per I2C; kein Kamerabild.
- Zielboard: Waveshare ESP32-S3-Zero, 4 MB Flash, native USB-OTG für HID.
- SDA GPIO 5, SCL GPIO 6, I2C-Adresse `0x58`, Bus 100 kHz.
- PlatformIO-Profil: `waveshare_s3_zero_4mb`, Plattform Espressif32 6.9.0, Arduino.
- Build: `bash scripts/build-release.sh`; erstellt Firmware und LittleFS und überschreibt ausschließlich `bootloader.bin`, `partitions.bin`, `boot_app0.bin`, `firmware.bin`, `littlefs.bin` im Projekt-Root.
- Flash-Adressen: Bootloader `0x0000`, Partitionstabelle `0x8000`, boot_app0 `0xE000`, Firmware `0x10000`, LittleFS `0x300000` (1 MB).

## Implementiert

- USB-HID-Touchscreen als Standardmodus und optionaler USB-Mausmodus.
- IR-Punkt aktiv hält Touch; IR-Punktverlust gibt Touch frei. Mausmodus bietet Klick, Drag-and-drop und einstellbaren Long-Press-Rechtsklick.
- Vier-Punkt-Homographie. Quell- und Zielpunkte, HID-Modus, Auflösung, Glättung, Blob-Mindestgröße, Empfindlichkeit, WLAN-Zugangsdaten und Long-Press-Zeit liegen im NVS.
- AP `IR-Whiteboard` mit Passwort `whiteboard` bleibt im AP+STA-Betrieb verfügbar; Web-UI unter `http://192.168.4.1`.
- Liveansicht hat zwei Koordinatenräume: Sensorfeld mit IR-Blobs und Kalibrier-Viereck; Bildschirm-Rechteck mit Zielpunkten und transformierter HID-Position.
- Kein Web-Flasher im Repository. Nur die fünf aktuellen Binärdateien und deren Flash-Offsets werden benötigt.

## Noch zu prüfen

- Firmware wurde mit PlatformIO kompiliert; reale USB-Erkennung und Touch-/Mausverhalten konnten mangels angeschlossener Hardware nicht getestet werden.
- Insbesondere HID-Touchscreen-Erkennung unter Windows/Android sowie Rechtsklick-/Drag-Gesten am konkreten Host prüfen.
- Sensorreichweite hängt von IR-LED, Optik, Ausrichtung und Umgebungslicht ab; eine maximale Distanz ist nicht verifiziert.
- Das AP-Passwort ist aktuell fest im Firmwarecode und sollte vor gemeinsam genutztem Einsatz änderbar gemacht werden.
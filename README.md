# ESP32-S3 IR Whiteboard

Neue PlatformIO-Firmware für das DFRobot SEN0158 IR-Positioning-Modul. Der Sensor verfolgt IR-LEDs und liefert bis zu vier Punktpositionen; er erzeugt **kein Videobild**. Die Weboberfläche zeigt deshalb Sensorpunkte, Kalibrierung und Verbindungsstatus live.

## Funktionen

- Native USB-HID-Touchscreen-Eingabe mit absoluter Position für Windows und Android.
- Touchscreen-Modus ist Standard und verwendet absolute Bildschirmkoordinaten. IR-LED an = Touch gedrückt; LED aus = Touch losgelassen. Verschieben bei aktiver LED ergibt Drag-and-drop; Doppeltippen und langes Drücken sind Touch-Gesten des Host-Betriebssystems.
- Optionaler USB-Mausmodus: kurzer Kontakt klickt, Bewegung bei aktivem IR-Punkt zieht, Doppelklick funktioniert über zwei Kontakte. Langes Halten löst nach einstellbarer Zeit einen Rechtsklick aus.
- Vier-Punkt-Homographie zur Ausrichtung auf das projizierte Bild.
- WLAN-Access-Point bleibt auch dann aktiv, wenn das Gerät zusätzlich einem vorhandenen WLAN beitritt.
- Web-Liveansicht, Sensorstatus, Touch-Test und konfigurierbare Filter.
- Einstellbare Bildschirmauflösung und persistente Konfiguration im ESP32-NVS.

Die Liveansicht zeigt zwei Koordinatenräume gleichzeitig: links die erkannten IR-Punkte im Sensorfeld mit dem aus vier Messpunkten gebildeten Kalibrier-Viereck; rechts das Bildschirm-Rechteck mit den vier Zielpunkten und der aktuell homographisch transformierten HID-Position. Die Einstellungen, die vier Kalibrierpaare, der HID-Schalter und WLAN-Zugangsdaten bleiben nach Neustarts im NVS gespeichert.

## Hardware und Pins

Zielboard: Waveshare ESP32-S3-Zero mit 4 MB Flash. Das Projekt enthält ein eigenes PlatformIO-Profil für genau dieses Ziel. Andere ESP32-S3-Boards brauchen ein passendes Boardprofil und eine Partitionstabelle für deren Flashgröße. Der USB-Anschluss muss native USB-OTG-Verbindung ermöglichen.

| SEN0158 | ESP32-S3 |
| --- | --- |
| VCC | 3V3 für den ESP32-S3 (DFRobot spezifiziert 3,3 V oder 5 V) |
| GND | GND |
| SDA | GPIO 5 |
| SCL | GPIO 6 |

Die 7-Bit-I²C-Adresse ist `0x58` (DFRobot-Beispiel: `0xB0 >> 1`), die Busgeschwindigkeit 100 kHz. Das offizielle Beispiel setzt Register `0x30`, `0x06`, `0x08`, `0x1A` und `0x33`, setzt den Datenzeiger auf `0x36` und liest 16 Bytes. Für die Koordinaten werden die Offsets `1/2/3`, `4/5/6`, `7/8/9` und `10/11/12` verwendet. Der Sketch nutzt zusätzlich zur DFRobot-Basisinitialisierung die sechs Empfindlichkeits-Presets aus dem alten Projekt; diese Presets sind nicht in DFRobots Beispielcode dokumentiert.

Quellen: [DFRobot SEN0158-Wiki und Arduino-Beispiel](https://wiki.dfrobot.com/sen0158/docs/20468), [SEN0158-Spezifikationen und Pinout](https://wiki.dfrobot.com/sen0158), [DFRobot I²C-Adressenliste](https://github.com/DFRobot/I2C_Addresses). DFRobot nennt 3,3 V/44 mA oder 5 V/48 mA, 0–3 m Erfassungsabstand sowie 33° horizontal und 23° vertikal. Für den ESP32-S3 ist die 3,3-V-Versorgung die vorsichtige Wahl; 5-V-I²C-Pegel nicht direkt an ESP32-GPIOs anschließen. IR-LED/Pen benötigt eine eigene passende Versorgung. Sensor und Projektionsfläche müssen sich im Sichtfeld befinden; direkte Sonne und andere starke IR-Quellen vermeiden. Das AP-Standardpasswort `whiteboard` ist für die Ersteinrichtung gedacht; ändere es vor einem Einsatz in gemeinsam genutzten Netzen.

## Inbetriebnahme

1. Installiere PlatformIO Core und Python 3.
2. Erzeuge Firmware, Dateisystem und Binärdateien mit `bash scripts/build-release.sh`.
3. Flash die fünf Binärdateien manuell mit den unten angegebenen Adressen.
4. Verbinde den Rechner/das Android-Gerät per USB-Datenkabel mit dem ESP32-S3. Der S3 muss native USB-OTG unterstützen; USB-UART-Adapter allein kann kein HID bereitstellen.
5. Verbinde dich mit WLAN `IR-Whiteboard`, Passwort `whiteboard`, und öffne `http://192.168.4.1`.
6. Stelle den IR-Sensor so auf, dass die gesamte Projektionsfläche sichtbar ist. Stelle im Webinterface die Bildschirmauflösung ein und erfasse die vier projizierten Ecken mit dem IR-Pen.
7. Aktiviere „USB-Touch senden“ und prüfe die Eingabe über „Touch testen“.

Der Access Point bleibt im AP+STA-Modus aktiv. Unter „WLAN verbinden“ gespeicherte Zugangsdaten verbinden das Gerät zusätzlich mit dem Heimnetz. Die IP-Adresse erscheint in der Statuszeile.

## Konfiguration

| Wert | Bedeutung |
| --- | --- |
| Bildschirmbreite/-höhe | Logische Touchscreen-Größe. Möglichst passend zur Auflösung des Projektors und der Windows-/Android-Anzeige setzen. |
| Mindestgröße IR-Punkt | Verwirft kleinere SEN0158-Blobs. Höher reduziert Störungen, kann schwache oder weit entfernte LEDs verlieren. |
| Sensorreichweite | SEN0158-Empfindlichkeitsstufe 1 bis 6. Stufe 6 ist am empfindlichsten und hilft bei größeren Abständen; geringere Stufen vermeiden Störungen bei nahen/starken IR-Punkten. |
| Glättung | EMA-Anteil der neuen Position. Niedrig = ruhiger, aber mehr Verzögerung; hoch = direkter, aber unruhiger. |
| Bewegung glätten | Schaltet den EMA-Filter ein/aus. |
| USB-Touch senden | Aktiviert/deaktiviert Touch-Berichte, ohne Webkonfiguration abzuschalten. |
| Eingabemodus | Touchscreen nutzt absolute Koordinaten; Mausmodus bewegt den relativen Cursor und bietet den konfigurierbaren Long-Press-Rechtsklick. |
| Rechtsklick ab | Haltezeit im Mausmodus, bevor ein Rechtsklick ausgegeben wird (350-2000 ms). |

Kalibrierung: öffne die Gerätekonfiguration auf dem Rechner, dessen Browserbild projiziert wird, und starte „Vollbild kalibrieren“. Richte den IR-Punkt nacheinander auf jedes projizierte Ziel und halte ihn dort ruhig; nach 2,2 Sekunden stabiler Erkennung wird die Ecke automatisch gespeichert und das nächste Ziel angezeigt. Zwischen Ecken muss der Punkt deutlich bewegt werden. Escape oder „Beenden“ bricht ab und gibt HID wieder frei. Browser-Vollbild wirkt nur auf dem Rechner, auf dem diese Seite geöffnet ist. Zum Nachjustieren kannst du nach abgeschlossener Kalibrierung die nummerierten Quellpunkte links im Sensorraum oder die Ziele rechts auf der Projektionsfläche ziehen. Die Homographie wird nach dem Loslassen neu berechnet und gespeichert; Polygon und Vorschau aktualisieren sich live.

Bei großer Sensor-Beamer-Entfernung den Sensor mittig und möglichst parallel zur Projektionsfläche ausrichten. Die Sensorreichweite hängt von IR-LED-Leistung, Optik, Winkel und Umgebungslicht ab; eine feste Maximaldistanz kann nicht garantiert werden. Eine engere IR-Optik am Sensor und eine kräftige, fokussierte IR-LED am Pen verbessern üblicherweise die Reichweite. Nach Änderung von Sensorposition, Zoom oder Projektionsfläche neu kalibrieren.

## Flash-Dateien und Adressen

Jeder Aufruf des Release-Build-Skripts erzeugt im Projekt-Root fünf Dateien mit gemeinsamem UTC-Zeitstempel, zum Beispiel `firmware-20261025-130000-UTC.bin`. Statische Aliasdateien und frühere Builds werden entfernt; es bleibt nur der aktuellste, zeitlich erkennbare Binärsatz.

| Datei | Adresse | Zweck |
| --- | ---: | --- |
| `bootloader-<timestamp>.bin` | `0x0000` | ESP32-S3-Startloader |
| `partitions-<timestamp>.bin` | `0x8000` | Partitionstabelle (NVS, OTA-Slot, LittleFS) |
| `boot_app0-<timestamp>.bin` | `0xE000` | Arduino-OTA-Metadatenbereich |
| `firmware-<timestamp>.bin` | `0x10000` | Programmcode |
| `littlefs-<timestamp>.bin` | `0x300000` | Weboberfläche im LittleFS-Dateisystem (1 MB) |

## Build

```sh
python3 -m pip install --user platformio
bash scripts/build-release.sh
```

Das Zielboard und die Pinbelegung werden in `platformio.ini` bzw. `src/main.cpp` festgelegt. Der frühere Arduino-Sketch im Projekt-Root und die Dateien unter `old_ESP32_cam/` bleiben als Referenz erhalten; gebaut wird ausschließlich `src/main.cpp`.
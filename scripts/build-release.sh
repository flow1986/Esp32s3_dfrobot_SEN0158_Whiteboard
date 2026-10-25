#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIO="${PLATFORMIO:-$HOME/.local/bin/pio}"
BUILD="$ROOT/.pio/build/waveshare-s3-zero"
FRAMEWORK="$HOME/.platformio/packages/framework-arduinoespressif32"

cd "$ROOT"
"$PIO" run -t buildfs
"$PIO" run

declare -A SOURCES=(
  [bootloader]="$BUILD/bootloader.bin"
  [partitions]="$BUILD/partitions.bin"
  [boot_app0]="$FRAMEWORK/tools/partitions/boot_app0.bin"
  [firmware]="$BUILD/firmware.bin"
  [littlefs]="$BUILD/littlefs.bin"
)

for name in bootloader partitions boot_app0 firmware littlefs; do
  source="${SOURCES[$name]}"
  if [[ ! -s "$source" ]]; then
    printf 'Build-Artefakt fehlt: %s\n' "$source" >&2
    exit 1
  fi
  cp "$source" "$ROOT/$name.bin"
done

find "$ROOT" -maxdepth 1 -type f \
  \( -name 'bootloader-*-UTC.bin' -o -name 'partitions-*-UTC.bin' \
  -o -name 'boot_app0-*-UTC.bin' -o -name 'firmware-*-UTC.bin' \
  -o -name 'littlefs-*-UTC.bin' \) -delete

printf 'Aktuelle Flash-Dateien wurden im Projekt-Root aktualisiert.\n'
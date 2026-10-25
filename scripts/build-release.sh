#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIO="${PLATFORMIO:-$HOME/.local/bin/pio}"
BUILD="$ROOT/.pio/build/waveshare-s3-zero"
FRAMEWORK="$HOME/.platformio/packages/framework-arduinoespressif32"
STAMP="$(date -u +%Y%m%d-%H%M%S-UTC)"

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
  cp "$source" "$ROOT/$name-$STAMP.bin"
  cp "$source" "$ROOT/$name.bin"
done

printf 'Build %s exportiert. Flash-Dateien liegen im Projekt-Root.\n' "$STAMP"
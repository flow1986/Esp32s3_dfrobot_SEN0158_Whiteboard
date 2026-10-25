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
done

for name in bootloader partitions boot_app0 firmware littlefs; do
  cp "${SOURCES[$name]}" "$ROOT/$name-$STAMP.bin"
done

rm -f "$ROOT/bootloader.bin" "$ROOT/partitions.bin" "$ROOT/boot_app0.bin" \
  "$ROOT/firmware.bin" "$ROOT/littlefs.bin"
find "$ROOT" -maxdepth 1 -type f \
  \( -name 'bootloader-*-UTC.bin' -o -name 'partitions-*-UTC.bin' \
  -o -name 'boot_app0-*-UTC.bin' -o -name 'firmware-*-UTC.bin' \
  -o -name 'littlefs-*-UTC.bin' \) ! -name "*-$STAMP.bin" -delete

printf 'Build %s exportiert. Nur der aktuelle Timestamp-Satz bleibt erhalten.\n' "$STAMP"
#!/usr/bin/env bash
set -euo pipefail

IDF_REF=25fe69f946311abdaf9ad56591f25fedbc20ac98
HERE="$(cd "$(dirname "$0")" && pwd)"
IDF_DIR="${IDF_DIR:-$HERE/.esp-idf}"

if [[ ! -d "$IDF_DIR/.git" ]]; then
    git clone https://github.com/espressif/esp-idf.git "$IDF_DIR"
fi

git -C "$IDF_DIR" fetch --depth 1 origin "$IDF_REF"
git -C "$IDF_DIR" checkout --detach "$IDF_REF"
git -C "$IDF_DIR" submodule update --init --recursive --depth 1
"$IDF_DIR/install.sh" esp32s3

source "$IDF_DIR/export.sh"
cd "$HERE"
rm -rf build-s3 sdkconfig.s3
idf.py -B build-s3 -DIDF_TARGET=esp32s3 \
    -DSDKCONFIG=sdkconfig.s3 \
    -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.esp32s3" build

mkdir -p firmware
cp build-s3/bootloader/bootloader.bin firmware/
cp build-s3/partition_table/partition-table.bin firmware/
cp build-s3/esp_sdr.bin firmware/
cp build-s3/esp_sdr.elf firmware/
cp build-s3/flasher_args.json firmware/
cp build-s3/flash_args firmware/ 2>/dev/null || true

cat > firmware/FLASH.txt <<'EOF'
Typical ESP32-S3 flash layout:
  bootloader.bin       @ 0x0000
  partition-table.bin  @ 0x8000
  esp_sdr.bin          @ 0x10000

Use the offsets from flasher_args.json if your ESP-IDF build emits different values.
EOF

echo
echo "Built firmware is in: $HERE/firmware"

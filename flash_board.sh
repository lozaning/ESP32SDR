#!/usr/bin/env bash
# Flash the built firmware to a specific board by its USB serial number
# (port names renumber on re-enumeration, the serial does not).
#
#   ./flash_board.sh 28:37:2F:98:5C:68        # flash the display firmware
#   ./flash_board.sh <serial> restore         # flash the validated no-display backup
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SN="${1:?usage: flash_board.sh <usb-serial> [restore]}"
SRC="$HERE/firmware"
[[ "${2:-}" == "restore" ]] && SRC="$HERE/firmware-rx-tx-validated-backup"

source "$HERE/.esp-idf/export.sh" >/dev/null 2>&1
PORT="$(python - "$SN" <<'PY'
import sys
from serial.tools import list_ports
sn = sys.argv[1]
for p in list_ports.comports():
    if (p.serial_number or "") == sn and p.device.startswith("/dev/cu."):
        print(p.device); break
PY
)"
[[ -n "${PORT:-}" ]] || { echo "no /dev/cu.* port for serial $SN (plugged in?)"; exit 1; }
echo "Flashing $SRC -> $SN ($PORT)"
python -m esptool --chip esp32s3 -p "$PORT" -b 460800 \
    --before usb-reset --after hard-reset write-flash \
    --flash-mode dio --flash-size 2MB --flash-freq 80m \
    0x0    "$SRC/bootloader.bin" \
    0x8000 "$SRC/partition-table.bin" \
    0x10000 "$SRC/esp_sdr.bin"
echo "Done."

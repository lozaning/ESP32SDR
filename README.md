# ESP32-S3 SDR link

A two-board software-defined-radio experiment on the ESP32-S3: one dongle is a
**receiver** (streaming an I/Q waterfall over USB), the other is a **transmitter**,
and together they carry **real messages over the air** — modulated, transmitted,
received, and decoded — driven from a web GUI.

It is built on the register-level "burst SDR" bypass that
[ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr) documented for reception,
extended here with a **working transmit path** that was reverse-engineered from
the vendor's `librftest.a` and RF-test firmware. It transmits and decodes
**OOK / ASK / 2-FSK / GFSK at 0 % BER** board-to-board.

> **What works:** RX I/Q capture + waterfall; OTA messaging via OOK/ASK/FSK/GFSK; a web GUI; an on-device ST7735 waveform display (T-Dongle-S3).
> **What does not work on this silicon:** BPSK/QPSK and arbitrary raw-I/Q transmit. They require the DAC-replay path, which — as verified exhaustively here — configures and triggers correctly but **does not reach the RF output** on the ESP32-S3 (it is an "undocumented debug path" that is not wired to the antenna). See [The transmit chain](#the-transmit-chain).

---

## Contents

```
CMakeLists.txt, main/, partitions.csv, sdkconfig.defaults*   ESP-IDF firmware (RX + TX + display)
build_s3.sh                 clone pinned ESP-IDF + build
flash_board.sh              flash a board by its USB serial (uses --before usb-reset)
firmware/                   prebuilt binaries (bootloader / partition-table / esp_sdr.bin)
gui/                        Python web GUI (Flask + WebSocket) : waterfall + message link
docs/FINDINGS.md            reverse-engineered register map
docs/vendor_trig_function.asm   annotated disassembly of the vendor trigger routine
```

---

## Hardware

- 2 × **LILYGO T-Dongle-S3** (ESP32-S3, 40 MHz crystal, native USB, 0.96" ST7735 LCD).
  Any ESP32-S3 board works for RX; TX uses only on-chip RF, no extra parts.
- The two boards must be able to hear each other. In free air a few cm apart, or
  in an RF-shielded enclosure with the antennas coupled, gives ~40–50 dB SNR at the RX.
- Everything (flashing, serial, **and JTAG**) runs over each board's **single
  built-in USB-Serial/JTAG** port — no external programmer or SDR is needed.

---

## The RF chain

The ESP32-S3 Wi-Fi radio has an undocumented debug path that connects the
baseband ADC/DAC to CPU-accessible SRAM, bypassing the fixed-function modem:

```
   CPU  <--->  SRAM I/Q buffer 0x3fcd0000 (64 KiB)  <--->  baseband  <--->  RF PLL / mixer  <--->  antenna
                     ^ capture engine (ADC -> SRAM)          works both directions in HW;
                     v replay  engine (SRAM -> DAC)          only the ADC->SRAM->CPU (RX) direction
                                                             is exposed to the antenna on the S3.
```

- **Receive** streams ADC samples into SRAM and the CPU reads them out — fully working.
- **Transmit** *should* stream SRAM → DAC → antenna, but on the S3 the DAC-replay
  half never radiates (see below). The working transmitter therefore uses the
  Wi-Fi PHY's own **calibration tone generator** as a controllable carrier and
  **keys/retunes it** to carry data (OOK / FSK).

### The receive chain (works)

`capture()` in `main/targets/esp32s3/receiver.c` drives the capture engine directly:

| Register | Meaning |
|---|---|
| `0x60033d5c` | capture control. `bit31`=enable, `bit19`=trigger pulse, `bit18`=done, `bit16/15`=rate, `bit17`=wide, `bits20-27`=source, `bits28-31`=mode, `low14`=sample count |
| `0x60033d90` | sample packing / AGC config |
| `0x600c101c` | SRAM owner (low 4 bits; **4** = capture engine owns the buffer) |
| `0x3fcd0000` | 64 KiB reserved I/Q aperture (`SOC_RESERVE_MEMORY_REGION`) |
| `0x6001c02c` | RX/TX gain (a.k.a. burst-gain register) |

Sequence: `prepare_rx()` puts the PHY in RX mode (`rom_pbus_xpd_rx_on`,
`rom_set_rxclk_en`, gain), then `capture()` sets the format reg, claims SRAM
(owner|4), writes the control word with `bit31`, pulses `bit19`, waits `bit18`.

**Sample rates** (RX, set by the rate divider): `0` ≈ 80 Msps (80 MHz span),
`1` ≈ 40 Msps, `6` ≈ 16 Msps.

**I/Q word format** (20 bits inside each 32-bit SRAM word):

```
bits [ 9: 0] = I   (signed 10-bit)
bits [19:10] = Q   (signed 10-bit)
```

`CAP16` packs each sample to two signed bytes by keeping the top 8 bits of each
field: `I = (word >> 2) & 0xFF`, `Q = (word >> 12) & 0xFF`.

### The transmit chain

Two paths exist; only the second radiates on the S3.

**1. DAC replay (`dactrig`, register `0x60033d64`) — does NOT radiate on S3.**
This is the "obvious" arbitrary-I/Q transmitter and the one the original
`tx_dac_replay()` targeted. The real `dactrig` is a linkable symbol in
`librftest.a`; its disassembly (see `docs/vendor_trig_function.asm` and
`docs/FINDINGS.md`) shows it: clears `bit31` of `0x60033d64`, fills `0x3fcd0000`
with a ramp, writes the sample count / clock (`bit15`) / flag (`bit19`) fields,
sets `bit31`, waits for `bit18`, then only calls `ets_delay_us` + `phy_printf`.
Calling the **real** `dactrig` with the PHY fully brought up, and injecting
arbitrary I/Q into the DAC descriptor ring over JTAG, both fail: the engine
completes (`done` bit sets) but **no commanded energy appears at the antenna** —
only fixed carrier/LO leakage. This matches the upstream README's note that the
firmware "currently implements reception only." The code is retained
(`TXIQ8`, `TXDMA`, `TXVENDOR`, and the `PEEK/POKE/TXLOAD` lab commands) for
anyone who wants to keep digging, but **do not expect RF from it.**

> **Update (2026-10-02):** the extra registers that `tx_dma()` copied from the
> vendor RF-test capture (`0x60033080`–`9c`, `0x60033d04`, `0x60033d08`, …) turned
> out to be **Wi-Fi MAC** receive-buffer and frame-transmit registers, not part of
> a DAC path. So chasing the "remaining suspects" in that capture is a dead end.
> See the correction section in [`docs/FINDINGS.md`](docs/FINDINGS.md).

**2. Carrier keying via the Wi-Fi tone generator — WORKS.**
`esp_phy_wifi_tx_tone()` (used by the vendor `wifiscwout` command) drives a
**digital tone generator in the Wi-Fi baseband** (two tone channels, each with a
software-set frequency and amplitude; `wifiscwout` uses one at frequency 0). It keys a strong
CW carrier on a Wi-Fi channel and radiates ~49 dB at the RX. `prepare_tx()` does:

```c
esp_phy_rftest_config(1); esp_phy_rftest_init();   // once
esp_phy_tx_contin_en(true);
esp_phy_wifi_tx_tone(1, chan, 0);                  // key the PA on `chan`
```

with `chan = (freq_mhz - 2412)/5 + 1`. The firmware exposes this as **`TXTONE 1`**
(key) / **`TXTONE 0`** (stop). The carrier snaps to the **5 MHz Wi-Fi channel
grid**, so frequency modulation is quantised to 5 MHz steps.

This carrier is controllable in **amplitude** (on/off, or gain) and **frequency**
(retune), which is exactly what amplitude and frequency modulations need:

| Modulation | Method | Status |
|---|---|---|
| OOK | carrier on / off | ✅ 0 % BER |
| ASK | carrier on / off (binary) | ✅ 0 % BER |
| 2-FSK | carrier keyed between two channels | ✅ 0 % BER |
| GFSK | same 2-FSK path (no Gaussian shaping) | ✅ 0 % BER |
| BPSK / QPSK | needs *phase* control | ❌ not implemented |
| raw I/Q | DAC replay does not radiate; baseband tone generator uncharacterised | ❌ not implemented |

### The message link (`gui/link.py`)

Host-orchestrated: the host keys the TX one bit at a time and reads the RX one
bit at a time, so timing is trivially synchronised (no clock recovery needed).

- **Frame:** `preamble 0xAA 0xAA` + `sync 0xF0` + `length` + `payload` + `CRC-8` (poly 0x07).
- **OOK/ASK:** per bit `TXTONE 1/0`; RX = total capture power vs a calibrated threshold.
- **2-FSK/GFSK:** `F0 = 2437 MHz`, `F1 = 2447 MHz` (10 MHz deviation, on the channel grid),
  RX tuned to `2442 MHz`; decide per bit by comparing energy in the F0 vs F1 band.

Two hardware quirks the decoder must handle (both learned the hard way):

1. **The RX spectrum is mirrored** (I/Q is conjugated), so a tone transmitted at
   `F` appears at apparent offset `RXC − F`, not `F − RXC`.
2. **The RX has a fixed internal spur at −3 MHz** from its tune center; keep both
   FSK tones off it, and set **`GAIN MANUAL ~22`** so the strong nearby carrier
   doesn't clip the 8-bit ADC.

---

## Build & flash

Requires macOS/Linux with Python 3 and git. The build script clones a **pinned**
ESP-IDF (commit `25fe69f946311abdaf9ad56591f25fedbc20ac98`) into `.esp-idf/` and
installs the ESP32-S3 toolchain — the first build downloads ~1 GB.

```bash
./build_s3.sh                       # clone pinned IDF, install toolchain, build -> build-s3/ and firmware/
```

Flash by **USB serial number** (the `/dev/cu.*` names renumber on re-enumeration;
the serial is stable). These USB-Serial/JTAG boards need `--before usb-reset`,
which `flash_board.sh` already uses:

```bash
./flash_board.sh 28:37:2F:98:5C:68  # flash the board with this USB serial
```

Find serials with `python -m esptool ... chip-id` or from the GUI's Dongles table.
To flash **prebuilt** binaries without building, point `flash_board.sh` at the
`firmware/` folder (bootloader @ `0x0`, partition-table @ `0x8000`, app @ `0x10000`).

### Build gotcha

If the source files carry **future modification times** (common after unzipping
across timezones), Ninja loops forever with *"build.ninja still dirty, perhaps
system time is not set."* Fix by normalising the timestamps before building:

```bash
find . -path ./.esp-idf -prune -o -newermt "$(date '+%Y-%m-%d %H:%M:%S')" -exec touch -h {} +
```

---

## Web GUI

```bash
cd gui && ./run.sh        # creates a venv on first run, serves http://127.0.0.1:8020
```

Requirements: `flask`, `flask-sock`, `pyserial`, `numpy` (see `gui/requirements.txt`).

1. In **Dongles**, click **RX** on one board and **TX** on another (both need this
   firmware; RX also works with stock ESPARGOS RX firmware).
2. **Receive · Waterfall** streams the live I/Q spectrum + envelope.
3. **Transmit · Message** — pick a modulation, type a message, **Send & Decode**.
   The generated waveform is drawn immediately; the transmission runs in the
   background (non-blocking) with live progress; the **decoded message, BER and
   CRC** appear under the waterfall, which keeps updating with the live captures
   during the send.

The GUI talks to the boards over the same USB serial protocol described below —
you can drive everything by hand with a serial terminal too.

---

## Serial command reference

Newline-terminated ASCII, one line in / one line out (binary payloads follow a
`READY`/`DATA` line). Highlights:

| Command | Effect |
|---|---|
| `INFO` | firmware id, e.g. `S3SDR-TX 7 burst 16380` |
| `CAPS` | capability list |
| `FREQ <mhz>` | tune (100–6000 MHz) |
| `CAP16 <n> <rate>` | capture `n` samples → `DATA <n> <crc32> <us>` then `n`×2 int8 I/Q bytes. rate 0/1/6 |
| `GAIN HARDWARE` \| `GAIN MANUAL <code>` | RX AGC vs fixed gain (use manual ~22 for the link) |
| **`TXTONE 1`** / **`TXTONE 0`** | key / stop the working CW carrier on the current channel |
| `RANGE?`, `LPF?`, `GAIN?` | queries |
| `TXIQ8`,`TXRAW`,`TXRAMP`,`TXDMA`,`TXVENDOR`,`TXLOAD*`,`PEEK`,`POKE` | experimental DAC-replay / lab commands (do not radiate — for RE only) |

---

## Reproducing the reverse engineering

Everything was done over the built-in USB-Serial/JTAG — no external tools.

- **Disassembly:** `xtensa-esp-elf-objdump` on the vendor RF-test `.bin`
  (`EspRFTestTool` → `Bin/RF_TEST_BIN/ESP32-S3_RFTest_*.bin`) and on
  `librftest.a` inside ESP-IDF (`components/esp_phy/lib/esp32s3/`). `dactrig`
  and `adctrig` are defined (`T`) symbols there — you can call or disassemble them.
- **Live registers:** `openocd -f board/esp32s3-builtin.cfg -c 'adapter serial "<MAC>"'`
  connects over the same USB; `mdw`/`mww` read/write peripheral registers while
  the firmware runs. The vendor firmware is a RAM app — `esptool ... load-ram`
  runs it without touching flash.
- **CLI injection without a UART:** the vendor console is on UART0 (GPIO43/44,
  not broken out to USB). You can still feed it commands by enabling UART0
  **loopback** (`0x60000020` bit14) and writing bytes to the FIFO (`0x60000000`)
  over JTAG — the firmware reads them as if received.
- **Detector:** the second board (this RX firmware) is the spectrum analyzer.
  Beware the −3 MHz spur, the mirrored spectrum, and ADC clipping (drop RX gain).

`docs/FINDINGS.md` has the full register map and the negative results, so nobody
has to rediscover that the DAC path (and the MAC registers mistaken for it) is a dead end.
The baseband tone generator behind `TXTONE` is the part of the TX chain known to
radiate, and is the open lead for anyone continuing transmit work.

---

## Credits

- [ESPARGOS/esp-sdr](https://github.com/ESPARGOS/esp-sdr) — the receive-side
  register bypass this builds on.
- Espressif `EspRFTestTool` / `librftest.a` — source of the `dactrig`/`adctrig`
  routines and the `esp_phy_*` tone/RF-test entry points.

## Safety / legal

The transmitter keys real RF in the 2.4 GHz band. Only transmit inside an
RF-shielded enclosure or under an appropriate license — do not radiate into
shared spectrum.

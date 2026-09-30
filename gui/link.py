"""Over-the-air message link between two ESP32-S3 boards.

Uses the working transmitter primitive (a controllable CW carrier: TXTONE 1/0
to key it, FREQ to retune it) that was reverse-engineered from the ESPARGOS
source (esp_phy_wifi_tx_tone). The DAC I/Q path does not radiate on the S3, so:

  OOK / ASK   -> carrier on/off (amplitude)          [works, 0% BER]
  2-FSK/GFSK  -> carrier keyed between two channels   [works, 0% BER]
  BPSK/QPSK   -> need phase control (DAC I/Q path)    [not possible on this HW]
  raw I/Q     -> DAC replay                           [not possible on this HW]

Host-orchestrated: the host keys the TX per bit and reads the RX per bit, so
timing is trivially synchronised. Frame = preamble + sync + len + payload + crc8.
"""
import numpy as np

OOK_FREQ = 2437                       # OOK carrier (MHz)
FSK_F0, FSK_F1, FSK_RXC = 2437, 2447, 2442   # 2-FSK tones + RX center (10 MHz dev)
PREAMBLE = [0xAA, 0xAA]
SYNC = [0xF0]
RX_GAIN = 22
SUPPORTED = {"ook", "ask", "fsk", "gfsk"}
PHASE_ONLY = {"bpsk", "qpsk", "raw"}


def crc8(data):
    c = 0
    for x in data:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def _bits(bs):
    return [(byte >> (7 - i)) & 1 for byte in bs for i in range(8)]


def _frame_bits(payload):
    body = [len(payload)] + list(payload)
    body.append(crc8(body))
    return _bits(PREAMBLE + SYNC + body), body


def _power(rx, avg, on_capture=None):
    vals = []
    while len(vals) < avg:
        z = rx.capture(n=16380, rate=0)
        if on_capture:
            try: on_capture(z)
            except Exception: pass
        vals.append(float(np.mean(np.abs(z - z.mean()) ** 2)))
    return 10 * np.log10(np.median(vals) + 1e-12)


def _fsk_bit(rx, b0, b1, avg, on_capture=None):
    scores = []
    while len(scores) < avg:
        z = rx.capture(n=16380, rate=0)
        if on_capture:
            try: on_capture(z)
            except Exception: pass
        z = z - z.mean()
        sp = np.abs(np.fft.fftshift(np.fft.fft(z * np.hanning(len(z))))) ** 2
        scores.append(sp[b1].sum() - sp[b0].sum())
    return 1 if np.median(scores) > 0 else 0


def build_preview(message, modulation, points=256):
    """Return the generated TX signal for display (no hardware): {scheme, bits, waveform}."""
    mod = modulation.lower()
    if mod in PHASE_ONLY:
        raise ValueError(f"{modulation.upper()} not supported on this hardware")
    payload = message.encode("utf-8", "replace")[:30]
    frame, _ = _frame_bits(payload)
    sps = 24
    b = np.repeat(np.array(frame, dtype=float), sps)
    t = np.arange(len(b))
    if mod in ("ook", "ask"):                      # gated carrier -> on/off bursts
        disp = np.abs(b * np.cos(2 * np.pi * t / 4.0))
    else:                                          # FSK: continuous-phase two-tone
        inst = np.where(b > 0, 1 / 3.0, 1 / 6.0)
        disp = (np.cos(2 * np.pi * np.cumsum(inst)) + 1) / 2
    m = len(disp) // points or 1
    disp = disp[:m * points].reshape(points, -1).max(axis=1)
    disp = np.clip(disp / (disp.max() or 1) * 255, 0, 255).astype(np.uint8)
    scheme = "OOK" if mod in ("ook", "ask") else "2-FSK"
    return {"scheme": scheme, "bits": "".join(map(str, frame)), "waveform": disp.tolist()}


def _decode(rxbits):
    """Find the sync word, read length + payload, verify CRC8."""
    patt = _bits(PREAMBLE + SYNC)
    L = len(patt)
    for off in range(0, max(1, len(rxbits) - L - 16)):
        if rxbits[off:off + L] == patt:
            p = off + L
            if p + 8 > len(rxbits):
                return None
            ln = int(np.packbits(rxbits[p:p + 8])[0]); p += 8
            if ln < 1 or p + ln * 8 + 8 > len(rxbits):
                continue
            body_bits = rxbits[p:p + ln * 8]; p += ln * 8
            payload = list(np.packbits(body_bits)) if body_bits else []
            rc = int(np.packbits(rxbits[p:p + 8])[0])
            if crc8([ln] + payload) == rc:
                return bytes(payload)
    # CRC not found: return best-effort payload after sync so the UI shows something
    for off in range(0, max(1, len(rxbits) - L - 16)):
        if rxbits[off:off + L] == patt:
            p = off + L
            if p + 8 <= len(rxbits):
                ln = int(np.packbits(rxbits[p:p + 8])[0]); p += 8
                bb = rxbits[p:p + ln * 8]
                if bb:
                    return bytes(np.packbits(bb)),
    return None


def transmit_receive(tx, rx, message, modulation, avg=3, progress=None, on_capture=None):
    """Send `message` from tx to rx using `modulation`; decode on rx. Returns dict.
    `progress(done, total)` after each bit; `on_capture(z)` for each RX snapshot
    (lets the caller keep the waterfall live during the transmission)."""
    def _tick(i, n):
        if progress:
            try: progress(i, n)
            except Exception: pass
    mod = modulation.lower()
    if mod in PHASE_ONLY:
        raise ValueError(
            f"{modulation.upper()} needs I/Q phase control (the DAC replay path), "
            "which does not radiate on the ESP32-S3. Use OOK/ASK or FSK/GFSK.")
    if mod not in SUPPORTED:
        raise ValueError(f"unknown modulation {modulation!r}")

    payload = message.encode("utf-8", "replace")
    if len(payload) > 30:
        payload = payload[:30]
    frame, body = _frame_bits(payload)

    rx.command("GAIN MANUAL %d" % RX_GAIN)
    rxb = []
    try:
        if mod in ("ook", "ask"):
            tx.set_freq(OOK_FREQ); rx.set_freq(OOK_FREQ)
            tx.command("TXTONE 1"); on = _power(rx, avg, on_capture)
            tx.command("TXTONE 0"); off = _power(rx, avg, on_capture)
            thr = (on + off) / 2.0
            for i, b in enumerate(frame):
                tx.command("TXTONE %d" % b)
                rxb.append(1 if _power(rx, avg, on_capture) > thr else 0)
                _tick(i + 1, len(frame))
            tx.command("TXTONE 0")
            info = {"scheme": "OOK", "carrier_mhz": OOK_FREQ,
                    "on_dB": round(on, 1), "off_dB": round(off, 1)}
        else:  # fsk / gfsk
            rx.set_freq(FSK_RXC)
            f = np.fft.fftshift(np.fft.fftfreq(16380, 1 / 80e6))
            # RX spectrum is mirrored (I/Q conjugated): apparent = RXC - F_tx
            b0 = np.abs(f - (FSK_RXC - FSK_F0) * 1e6) < 1.2e6
            b1 = np.abs(f - (FSK_RXC - FSK_F1) * 1e6) < 1.2e6
            tx.command("TXTONE 0")
            for i, b in enumerate(frame):
                tx.set_freq(FSK_F1 if b else FSK_F0)
                tx.command("TXTONE 1")
                rxb.append(_fsk_bit(rx, b0, b1, avg, on_capture))
                _tick(i + 1, len(frame))
            tx.command("TXTONE 0")
            info = {"scheme": "2-FSK", "f0_mhz": FSK_F0, "f1_mhz": FSK_F1,
                    "deviation_mhz": FSK_F1 - FSK_F0}
    finally:
        try:
            tx.command("TXTONE 0")
            rx.command("GAIN HARDWARE")
        except Exception:
            pass

    decoded = _decode(rxb)
    crc_ok = isinstance(decoded, bytes)
    if isinstance(decoded, tuple):        # best-effort (no CRC match)
        decoded = decoded[0]
    ber = sum(a != b for a, b in zip(frame, rxb)) / max(1, len(frame))
    text = None
    if decoded is not None:
        text = decoded.decode("utf-8", "replace")
    return {
        "sent": message, "decoded": text, "crc_ok": crc_ok,
        "ber_pct": round(ber * 100, 2), "bit_errors": sum(a != b for a, b in zip(frame, rxb)),
        "total_bits": len(frame), "matched": (text == message),
        "sent_bits": "".join(map(str, frame)), "recv_bits": "".join(map(str, rxb)),
        "info": info,
    }

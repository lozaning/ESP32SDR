"""Baseband I/Q synthesis for the ESP-SDR S3 transmitter.

The firmware only replays a finite buffer of signed int8 I/Q samples
(<= 16380) at the DAC rate, so all modulation happens here on the host: we
build a complex baseband waveform, shift it to a chosen IF offset (to clear
LO leakage at the carrier), scale to int8 and hand it to TXIQ8.
"""
import numpy as np

from devices import MAX_TX_SAMPLES, TX_FS

AMP = 100          # int8 peak (headroom below 127 -> 10-bit DAC via *4)


def bits_from_text(s):
    return np.unpackbits(np.frombuffer(s.encode("utf-8", "replace"), dtype=np.uint8))


def bits_from_hex(s):
    s = "".join(s.split()).replace("0x", "")
    if len(s) % 2:
        s = "0" + s
    b = bytes.fromhex(s) if s else b""
    return np.unpackbits(np.frombuffer(b, dtype=np.uint8))


def _bits(data, fmt):
    if fmt == "hex":
        return bits_from_hex(data)
    return bits_from_text(data)


def _preamble_bits(preamble_hex):
    if not preamble_hex.strip():
        return np.array([], dtype=np.uint8)
    return bits_from_hex(preamble_hex)


def _finalize(iq, fs, offset_hz):
    """Shift to IF offset, normalize to int8, clamp to MAX_TX_SAMPLES."""
    iq = iq[:MAX_TX_SAMPLES]
    n = len(iq)
    if n == 0:
        raise ValueError("no samples produced (empty data?)")
    t = np.arange(n) / fs
    iq = iq * np.exp(2j * np.pi * offset_hz * t)
    peak = np.max(np.abs(iq)) or 1.0
    iq = iq / peak * AMP
    out = np.empty(n * 2, dtype=np.int8)
    out[0::2] = np.clip(np.round(iq.real), -127, 127).astype(np.int8)
    out[1::2] = np.clip(np.round(iq.imag), -127, 127).astype(np.int8)
    return out.tobytes()


def _sps(fs, baud):
    sps = int(round(fs / max(baud, 1)))
    return max(sps, 2)


def _gaussian_filter(x, bt, sps):
    """Gaussian pulse filter for GFSK; bt is bandwidth-time product."""
    span = 4
    n = span * sps
    t = (np.arange(-n, n + 1)) / sps
    alpha = np.sqrt(np.log(2) / 2) / bt
    h = np.exp(-(np.pi ** 2) * (t ** 2) / (alpha ** 2))
    h /= h.sum()
    return np.convolve(x, h, mode="same")


def _rrc(beta, sps, span=6):
    n = np.arange(-span * sps, span * sps + 1)
    t = n / sps
    h = np.zeros_like(t, dtype=float)
    for i, ti in enumerate(t):
        if abs(ti) < 1e-8:
            h[i] = 1 - beta + 4 * beta / np.pi
        elif beta > 0 and abs(abs(4 * beta * ti) - 1) < 1e-8:
            h[i] = (beta / np.sqrt(2)) * (
                (1 + 2 / np.pi) * np.sin(np.pi / (4 * beta))
                + (1 - 2 / np.pi) * np.cos(np.pi / (4 * beta)))
        else:
            h[i] = (np.sin(np.pi * ti * (1 - beta))
                    + 4 * beta * ti * np.cos(np.pi * ti * (1 + beta))) / (
                np.pi * ti * (1 - (4 * beta * ti) ** 2))
    return h / np.sqrt(np.sum(h ** 2))


def synthesize(scheme, data, fmt, fs, params):
    """Return (iq_bytes, meta). fs is the DAC replay rate for the chosen clock."""
    offset = float(params.get("offset_hz", 2e6))
    baud = float(params.get("baud", 1e6))

    if scheme == "raw":
        raw = "".join(data.split()).replace("0x", "")
        if len(raw) % 2:
            raw = "0" + raw
        b = bytes.fromhex(raw) if raw else b""
        if len(b) % 2:
            b = b[:-1]
        b = b[: MAX_TX_SAMPLES * 2]
        if not b:
            raise ValueError("raw I/Q hex is empty")
        arr = np.frombuffer(b, dtype=np.int8)
        return bytes(arr.tobytes()), {
            "scheme": "raw", "samples": len(b) // 2, "fs": fs}

    bits = np.concatenate([_preamble_bits(params.get("preamble", "")),
                           _bits(data, fmt)]).astype(int)
    repeat = int(params.get("repeat", 1))
    if repeat > 1:
        bits = np.tile(bits, repeat)
    if len(bits) == 0:
        raise ValueError("no bits to send")
    sps = _sps(fs, baud)

    if scheme in ("ook", "ask"):
        depth = float(params.get("depth", 0.0))  # 0 = pure OOK
        levels = np.where(bits > 0, 1.0, depth)
        env = np.repeat(levels, sps)
        iq = env.astype(complex)
        meta_extra = {"depth": depth}

    elif scheme in ("fsk", "gfsk"):
        dev = float(params.get("deviation", 500e3))
        nrz = np.repeat(2 * bits - 1, sps).astype(float)
        if scheme == "gfsk":
            nrz = _gaussian_filter(nrz, float(params.get("bt", 0.5)), sps)
        phase = 2 * np.pi * dev * np.cumsum(nrz) / fs
        iq = np.exp(1j * phase)
        meta_extra = {"deviation_hz": dev}

    elif scheme in ("bpsk", "qpsk"):
        beta = float(params.get("rrc_beta", 0.35))
        if scheme == "bpsk":
            syms = (2 * bits - 1).astype(complex)
        else:
            if len(bits) % 2:
                bits = np.append(bits, 0)
            pairs = bits.reshape(-1, 2)
            m = {(0, 0): 1 + 1j, (0, 1): -1 + 1j,
                 (1, 1): -1 - 1j, (1, 0): 1 - 1j}
            syms = np.array([m[tuple(p)] for p in pairs]) / np.sqrt(2)
        up = np.zeros(len(syms) * sps, dtype=complex)
        up[::sps] = syms
        iq = np.convolve(up, _rrc(beta, sps), mode="same")
        meta_extra = {"rrc_beta": beta, "order": 2 if scheme == "bpsk" else 4}

    else:
        raise ValueError(f"unknown scheme {scheme!r}")

    truncated = len(iq) > MAX_TX_SAMPLES
    iq_bytes = _finalize(iq, fs, offset)
    meta = {"scheme": scheme, "bits": int(len(bits)), "sps": sps,
            "baud": baud, "offset_hz": offset, "fs": fs,
            "samples": len(iq_bytes) // 2, "truncated": truncated}
    meta.update(meta_extra)
    return iq_bytes, meta

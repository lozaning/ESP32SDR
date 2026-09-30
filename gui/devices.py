"""Serial device manager for ESP-SDR S3 dongles.

Each board is addressed by its stable USB serial number, never by the
/dev/cu.* name (those renumber on re-enumeration). One SdrDevice owns its
port exclusively and is driven from a single worker thread, so the firmware's
5-second command lease is never contended.
"""
import threading
import time
import zlib

import numpy as np
import serial
from serial.tools import list_ports

# Effective sample rates (Hz), derived from measured capture/replay timing.
RX_FS = {0: 80e6, 1: 40e6, 6: 16e6}
TX_FS = {0: 80e6, 1: 40e6}
MAX_TX_SAMPLES = 16380


def list_boards():
    """Return [{serial, port}] for every plausible ESP32-S3 CDC/JTAG port."""
    out = []
    for p in list_ports.comports():
        dev = p.device
        if not dev.startswith("/dev/cu."):
            continue
        if "Bluetooth" in dev or "debug-console" in dev or "wlan" in dev:
            continue
        # ESP32-S3 native USB serial/JTAG is VID 0x303a; also accept any
        # port that carries a serial number (covers UART bridges).
        sn = p.serial_number
        if p.vid == 0x303A or sn:
            out.append({"serial": sn or dev, "port": dev,
                        "vid": p.vid, "pid": p.pid,
                        "description": p.description})
    return out


def _port_for_serial(sn):
    for p in list_ports.comports():
        if (p.serial_number == sn or p.device == sn) and p.device.startswith("/dev/cu."):
            return p.device
    return None


class SdrError(Exception):
    pass


class SdrDevice:
    def __init__(self, serial_number):
        self.serial_number = serial_number
        self.port = None
        self._ser = None
        self._lock = threading.Lock()
        self.info = ""
        self.caps = ""
        self.tx_capable = False
        self.freq_mhz = 2437

    # ---- lifecycle -------------------------------------------------
    def open(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                return
            self.port = _port_for_serial(self.serial_number)
            if not self.port:
                raise SdrError(f"device {self.serial_number} not present")
            self._ser = serial.Serial(self.port, 115200, timeout=3)
            time.sleep(0.2)
            self._ser.reset_input_buffer()
        self.info = self._cmd("INFO")
        self.caps = self._cmd("CAPS")
        self.tx_capable = ("TX" in self.info) or ("TXIQ8" in self.caps)

    def close(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(b"RELEASE\n")
                    self._ser.readline()
                except Exception:
                    pass
                self._ser.close()
            self._ser = None

    def is_open(self):
        return bool(self._ser and self._ser.is_open)

    # ---- low level -------------------------------------------------
    def _readline(self):
        return self._ser.readline().decode(errors="replace").rstrip()

    def _read_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self._ser.read(n - len(buf))
            if not chunk:
                break
            buf += chunk
        return bytes(buf)

    def _cmd(self, cmd, expect=None):
        with self._lock:
            if not (self._ser and self._ser.is_open):
                raise SdrError("port closed")
            self._ser.reset_input_buffer()
            self._ser.write((cmd + "\n").encode())
            line = self._readline()
        if expect and not line.startswith(expect):
            raise SdrError(f"{cmd!r} -> {line!r}")
        return line

    def command(self, cmd, expect=None):
        return self._cmd(cmd, expect)

    # ---- high level ------------------------------------------------
    def set_freq(self, mhz):
        mhz = int(mhz)
        self._cmd(f"FREQ {mhz}", expect="OK")
        self.freq_mhz = mhz

    def capture(self, n=4096, rate=0):
        """Return complex64 array of n samples (int8 I/Q from CAP16)."""
        n = int(n)
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(f"CAP16 {n} {rate}\n".encode())
            header = self._readline().split()
            if not header or header[0] != "DATA":
                raise SdrError(f"capture failed: {' '.join(header) or 'no reply'}")
            data = self._read_exact(n * 2)
        if len(data) != n * 2:
            raise SdrError(f"short read {len(data)}/{n*2}")
        if zlib.crc32(data) & 0xFFFFFFFF != int(header[2], 16):
            raise SdrError("capture CRC mismatch")
        a = np.frombuffer(data, dtype=np.int8).astype(np.float32).reshape(-1, 2)
        return a[:, 0] + 1j * a[:, 1]

    def transmit(self, iq_bytes, clock=0, flag=0, layout=0):
        """Upload signed interleaved int8 I/Q and replay it once."""
        if not self.tx_capable:
            raise SdrError("device is not TX-capable")
        nbytes = len(iq_bytes)
        if nbytes % 2:
            raise SdrError("I/Q byte count must be even")
        n = nbytes // 2
        if not (1 <= n <= MAX_TX_SAMPLES):
            raise SdrError(f"sample count {n} out of range 1..{MAX_TX_SAMPLES}")
        crc = zlib.crc32(iq_bytes) & 0xFFFFFFFF
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(f"TXIQ8 {n} {clock} {flag} {layout} {crc:08x}\n".encode())
            r = self._readline()
            if not r.startswith("READY"):
                raise SdrError(f"TX not ready: {r}")
            self._ser.write(iq_bytes)
            self._ser.flush()
            done = self._readline()
        if not done.startswith("TXDONE"):
            raise SdrError(f"TX failed: {done}")
        parts = done.split()
        return {"samples": int(parts[1]), "elapsed_us": int(parts[2])}


class DeviceManager:
    """Holds role assignments and open devices for the whole session."""
    def __init__(self):
        self.devices = {}      # serial -> SdrDevice
        self.roles = {}        # serial -> 'rx' | 'tx'
        self._lock = threading.Lock()

    def scan(self):
        boards = list_boards()
        present = {b["serial"] for b in boards}
        for b in boards:
            sn = b["serial"]
            dev = self.devices.get(sn)
            b["role"] = self.roles.get(sn)
            b["open"] = bool(dev and dev.is_open())
            b["info"] = dev.info if dev else ""
            b["tx_capable"] = dev.tx_capable if dev else None
            b["freq_mhz"] = dev.freq_mhz if dev else None
        # forget devices that unplugged
        for sn in list(self.devices):
            if sn not in present:
                try:
                    self.devices[sn].close()
                except Exception:
                    pass
        return boards

    def get(self, sn):
        with self._lock:
            dev = self.devices.get(sn)
            if not dev:
                dev = SdrDevice(sn)
                self.devices[sn] = dev
            if not dev.is_open():
                dev.open()
            return dev

    def assign(self, sn, role):
        if role not in ("rx", "tx", None):
            raise SdrError("role must be rx, tx or null")
        if role is None:
            self.roles.pop(sn, None)
            dev = self.devices.get(sn)
            if dev:
                dev.close()
            return {"serial": sn, "role": None}
        dev = self.get(sn)
        if role == "tx" and not dev.tx_capable:
            raise SdrError(f"{dev.info or sn} is not TX-capable (RX-only firmware)")
        self.roles[sn] = role
        return {"serial": sn, "role": role, "info": dev.info,
                "tx_capable": dev.tx_capable}

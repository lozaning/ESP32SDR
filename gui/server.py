"""ESP-SDR S3 web GUI.

    python server.py            # http://127.0.0.1:8000

Left half: assign dongles to RX/TX, live waterfall + waveform from the RX
board. Right half: enter data, pick a modulation, transmit from the TX board.
"""
import json
import queue
import threading
import time

import numpy as np
from flask import Flask, jsonify, request, send_from_directory
from flask_sock import Sock

import devices
import modulation
import link

app = Flask(__name__, static_folder="static", static_url_path="")
sock = Sock(app)
mgr = devices.DeviceManager()

WATERFALL_BINS = 256
WAVE_POINTS = 256


def _reduce(arr, m):
    """Map any-length array to exactly m points (max-pool if longer, interp if not)."""
    arr = np.asarray(arr, dtype=float)
    if len(arr) == 0:
        return np.zeros(m)
    if len(arr) >= m:
        trim = (len(arr) // m) * m
        return arr[:trim].reshape(m, -1).max(axis=1)
    xp = np.linspace(0, 1, len(arr))
    return np.interp(np.linspace(0, 1, m), xp, arr)

_clients = set()
_clients_lock = threading.Lock()
_rx_threads = {}          # serial -> RxWorker


def broadcast(obj):
    msg = json.dumps(obj)
    with _clients_lock:
        dead = []
        for q in _clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                dead.append(q)
        for q in dead:
            _clients.discard(q)


class RxWorker(threading.Thread):
    def __init__(self, serial_number):
        super().__init__(daemon=True)
        self.sn = serial_number
        self.stop_flag = threading.Event()
        self.n = 4096
        self.rate = 0
        self.lock = threading.Lock()

    def configure(self, n=None, rate=None):
        with self.lock:
            if n:
                self.n = int(n)
            if rate is not None:
                self.rate = int(rate)

    def run(self):
        while not self.stop_flag.is_set():
            try:
                dev = mgr.get(self.sn)
                with self.lock:
                    n, rate = self.n, self.rate
                z = dev.capture(n=n, rate=rate)
            except Exception as e:
                broadcast({"type": "rx_error", "serial": self.sn, "error": str(e)})
                # self-heal: drop the handle so the next loop reopens a fresh
                # port (recovers from transient USB-CDC glitches / contention).
                try:
                    mgr.get(self.sn).close()
                except Exception:
                    pass
                time.sleep(0.6)
                continue
            fs = devices.RX_FS.get(rate, 80e6)
            frame = self._analyze(z, fs, dev.freq_mhz)
            frame.update({"type": "rx", "serial": self.sn})
            broadcast(frame)
            time.sleep(0.03)

    def _analyze(self, z, fs, center_mhz):
        z = z - z.mean()
        win = np.hanning(len(z))
        sp = _reduce(np.fft.fftshift(np.abs(np.fft.fft(z * win))), WATERFALL_BINS)
        power = 20 * np.log10(sp + 1e-6)
        lo, hi = np.percentile(power, 5), np.percentile(power, 99)
        row = np.clip((power - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(np.uint8)
        # time-domain magnitude envelope
        mag = np.abs(z)
        env = _reduce(mag, WAVE_POINTS)
        env = np.clip(env / (np.max(env) or 1) * 255, 0, 255).astype(np.uint8)
        return {
            "waterfall": row.tolist(),
            "waveform": env.tolist(),
            "center_mhz": center_mhz,
            "span_mhz": fs / 1e6,
            "rms": float(np.sqrt(np.mean(mag ** 2))),
        }


def _preview(iq_bytes):
    a = np.frombuffer(iq_bytes, dtype=np.int8).astype(float).reshape(-1, 2)
    z = a[:, 0] + 1j * a[:, 1]
    env = _reduce(np.abs(z), WAVE_POINTS)
    env = np.clip(env / (np.max(env) or 1) * 255, 0, 255).astype(np.uint8)
    sp = _reduce(np.fft.fftshift(np.abs(np.fft.fft(z * np.hanning(len(z))))), WATERFALL_BINS)
    power = 20 * np.log10(sp + 1e-6)
    lo, hi = power.min(), power.max()
    spec = np.clip((power - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(np.uint8)
    return {"waveform": env.tolist(), "spectrum": spec.tolist()}


# ---------------- HTTP API ----------------
@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@app.route("/api/boards")
def api_boards():
    try:
        return jsonify(mgr.scan())
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/assign", methods=["POST"])
def api_assign():
    d = request.get_json(force=True)
    sn, role = d.get("serial"), d.get("role")
    try:
        res = mgr.assign(sn, role)
    except Exception as e:
        return jsonify({"error": str(e)}), 400
    # manage the RX worker for this device
    w = _rx_threads.get(sn)
    if role == "rx":
        if not w or not w.is_alive():
            w = RxWorker(sn)
            _rx_threads[sn] = w
            w.start()
    else:
        if w:
            w.stop_flag.set()
            _rx_threads.pop(sn, None)
    return jsonify(res)


@app.route("/api/rx/config", methods=["POST"])
def api_rx_config():
    d = request.get_json(force=True)
    sn = d.get("serial")
    try:
        if d.get("freq"):
            mgr.get(sn).set_freq(d["freq"])
        w = _rx_threads.get(sn)
        if w:
            w.configure(n=d.get("n"), rate=d.get("rate"))
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/tx/preview", methods=["POST"])
def api_tx_preview():
    d = request.get_json(force=True)
    try:
        clock = int(d.get("clock", 0))
        fs = devices.TX_FS.get(clock, 80e6)
        iq, meta = modulation.synthesize(
            d.get("scheme", "ook"), d.get("data", ""), d.get("format", "text"),
            fs, d.get("params", {}))
        pv = _preview(iq)
        pv.update({"meta": meta})
        return jsonify(pv)
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/tx/send", methods=["POST"])
def api_tx_send():
    d = request.get_json(force=True)
    sn = d.get("serial")
    try:
        dev = mgr.get(sn)
        if d.get("freq"):
            dev.set_freq(d["freq"])
        clock = int(d.get("clock", 0))
        layout = int(d.get("layout", 0))
        fs = devices.TX_FS.get(clock, 80e6)
        iq, meta = modulation.synthesize(
            d.get("scheme", "ook"), d.get("data", ""), d.get("format", "text"),
            fs, d.get("params", {}))
        result = dev.transmit(iq, clock=clock, flag=0, layout=layout)
        pv = _preview(iq)
        broadcast({"type": "tx", "serial": sn, "meta": meta, "result": result,
                   "waveform": pv["waveform"], "spectrum": pv["spectrum"],
                   "freq_mhz": dev.freq_mhz})
        return jsonify({"ok": True, "result": result, "meta": meta,
                        "preview": pv, "freq_mhz": dev.freq_mhz})
    except Exception as e:
        return jsonify({"error": str(e)}), 400


_link_busy = threading.Lock()


def _run_link(tx_sn, rx_sn, msg, mod, avg):
    """Runs in a background thread; streams progress + result over the WebSocket."""
    # pause the RX worker so the link has exclusive, synchronous board access
    w = _rx_threads.get(rx_sn)
    if w:
        w.stop_flag.set(); w.join(timeout=2); _rx_threads.pop(rx_sn, None)
    try:
        tx = mgr.get(tx_sn); rx = mgr.get(rx_sn)
        def prog(done, total):
            broadcast({"type": "link_progress", "done": done, "total": total})
        cap_n = [0]
        def oncap(z):
            cap_n[0] += 1
            if cap_n[0] % 3:                 # throttle so the waterfall stays smooth
                return
            z = z - z.mean()
            sp = _reduce(np.fft.fftshift(np.abs(np.fft.fft(z * np.hanning(len(z))))), WATERFALL_BINS)
            power = 20 * np.log10(sp + 1e-6)
            lo, hi = np.percentile(power, 5), np.percentile(power, 99)
            row = np.clip((power - lo) / max(hi - lo, 1e-6) * 255, 0, 255).astype(np.uint8)
            env = _reduce(np.abs(z), WAVE_POINTS)
            env = np.clip(env / (np.max(env) or 1) * 255, 0, 255).astype(np.uint8)
            broadcast({"type": "rx", "serial": rx_sn, "waterfall": row.tolist(),
                       "waveform": env.tolist(), "center_mhz": rx.freq_mhz, "span_mhz": 80.0,
                       "rms": float(np.sqrt(np.mean(np.abs(z) ** 2)))})
        result = link.transmit_receive(tx, rx, msg, mod, avg=avg, progress=prog, on_capture=oncap)
    except Exception as e:
        result = {"error": str(e)}
    finally:
        if mgr.roles.get(rx_sn) == "rx":
            nw = RxWorker(rx_sn); _rx_threads[rx_sn] = nw; nw.start()
        _link_busy.release()
    broadcast({"type": "link", **result})


@app.route("/api/link/preview", methods=["POST"])
def api_link_preview():
    """Generated TX waveform for a message+modulation (no hardware)."""
    d = request.get_json(force=True)
    try:
        return jsonify(link.build_preview(d.get("message", ""), d.get("modulation", "ook")))
    except Exception as e:
        return jsonify({"error": str(e)}), 400


@app.route("/api/link/send", methods=["POST"])
def api_link_send():
    """Kick off an OTA message link in the background; result arrives over the WS."""
    d = request.get_json(force=True)
    tx_sn = d.get("tx") or next((s for s, r in mgr.roles.items() if r == "tx"), None)
    rx_sn = d.get("rx") or next((s for s, r in mgr.roles.items() if r == "rx"), None)
    if not tx_sn or not rx_sn:
        return jsonify({"error": "assign one board to TX and one to RX first"}), 400
    if tx_sn == rx_sn:
        return jsonify({"error": "TX and RX must be different boards"}), 400
    if not _link_busy.acquire(blocking=False):
        return jsonify({"error": "a transmission is already in progress"}), 409
    threading.Thread(target=_run_link, args=(tx_sn, rx_sn, d.get("message", ""),
                     d.get("modulation", "ook"), int(d.get("avg", 3))),
                     daemon=True).start()
    return jsonify({"started": True})


@sock.route("/ws")
def ws(ws):
    q = queue.Queue(maxsize=64)
    with _clients_lock:
        _clients.add(q)
    try:
        while True:
            try:
                msg = q.get(timeout=30)
            except queue.Empty:
                ws.send(json.dumps({"type": "ping"}))
                continue
            ws.send(msg)
    except Exception:
        pass
    finally:
        with _clients_lock:
            _clients.discard(q)


if __name__ == "__main__":
    import os
    port = int(os.environ.get("PORT", "8020"))
    print(f"ESP-SDR S3 GUI -> http://127.0.0.1:{port}")
    app.run(host="127.0.0.1", port=port, threaded=True)

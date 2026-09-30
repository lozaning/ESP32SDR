"use strict";
const $ = (id) => document.getElementById(id);
const api = (url, body) =>
  fetch(url, body ? {method: "POST", headers: {"Content-Type": "application/json"},
        body: JSON.stringify(body)} : undefined).then(r => r.json());

// ---- device table ----
let boards = [];
async function refresh() {
  boards = await api("/api/boards");
  if (boards.error) { setStatus("scan error: " + boards.error, false); return; }
  const tb = $("devtable").querySelector("tbody");
  tb.innerHTML = "";
  for (const b of boards) {
    const tr = document.createElement("tr");
    const fw = b.info || (b.open ? "?" : "(not opened)");
    tr.innerHTML = `<td>${b.serial}</td><td>${fw}</td><td>${b.port.replace("/dev/cu.", "")}</td>`;
    const td = document.createElement("td");
    for (const role of ["rx", "tx"]) {
      const btn = document.createElement("button");
      btn.className = "role ghost" + (b.role === role ? " on" : "");
      btn.textContent = role.toUpperCase();
      btn.onclick = () => assign(b.serial, b.role === role ? null : role);
      td.appendChild(btn);
    }
    tr.appendChild(td);
    tb.appendChild(tr);
  }
  fillRoleSelects();
}
function fillRoleSelects() {
  for (const [sel, role] of [["rx-dev", "rx"], ["tx-dev", "tx"]]) {
    const el = $(sel); if (!el) continue;
    const cur = el.value;
    el.innerHTML = '<option value="">— none —</option>';
    boards.filter(b => b.role === role).forEach(b => {
      const o = document.createElement("option");
      o.value = b.serial; o.textContent = b.serial + (b.info ? " · " + b.info : "");
      el.appendChild(o);
    });
    if ([...el.options].some(o => o.value === cur)) el.value = cur;
    else if (el.options.length === 2) el.selectedIndex = 1;
  }
}
async function assign(serial, role) {
  const r = await api("/api/assign", {serial, role});
  if (r.error) { setStatus(r.error, false); alert(r.error); }
  await refresh();
}

// ---- RX rendering ----
const wf = $("waterfall"), wfx = wf.getContext("2d");
const rxw = $("rx-wave"), rxwx = rxw.getContext("2d");
function pushWaterfall(row) {
  const w = wf.width, h = wf.height;
  const img = wfx.getImageData(0, 0, w, h - 1);
  wfx.putImageData(img, 0, 1);
  const line = wfx.createImageData(w, 1);
  for (let x = 0; x < w; x++) {
    const v = row[Math.floor(x * row.length / w)] || 0;
    const [r, g, b] = viridis(v / 255);
    line.data[x * 4] = r; line.data[x * 4 + 1] = g; line.data[x * 4 + 2] = b; line.data[x * 4 + 3] = 255;
  }
  wfx.putImageData(line, 0, 0);
}
function drawWave(ctx, cvs, arr, color) {
  const w = cvs.width, h = cvs.height;
  ctx.fillStyle = "#05070a"; ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = "#1b2430"; ctx.beginPath(); ctx.moveTo(0, h / 2); ctx.lineTo(w, h / 2); ctx.stroke();
  ctx.strokeStyle = color; ctx.lineWidth = 1;
  for (const sgn of [-1, 1]) {
    ctx.beginPath();
    for (let x = 0; x < w; x++) {
      const v = arr[Math.floor(x * arr.length / w)] || 0;
      const y = h / 2 + sgn * (v / 255) * (h / 2 - 2);
      x ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    }
    ctx.stroke();
  }
}
function viridis(t) {
  t = Math.max(0, Math.min(1, t));
  const r = Math.round(255 * Math.min(1, 0.28 + 1.9 * t - 1.1 * t * t));
  const g = Math.round(255 * Math.min(1, 0.02 + 1.1 * t));
  const b = Math.round(255 * (0.33 + 0.7 * t - 1.0 * t * t));
  return [r, g, Math.max(40, b)];
}

// ---- websocket ----
let ws;
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => setStatus("connected", true);
  ws.onclose = () => { setStatus("disconnected — retrying", false); setTimeout(connect, 1500); };
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === "rx") {
      if ($("rx-dev").value && m.serial !== $("rx-dev").value) return;
      pushWaterfall(m.waterfall);
      drawWave(rxwx, rxw, m.waveform, "#3fb950");
      const lo = m.center_mhz - m.span_mhz / 2, hi = m.center_mhz + m.span_mhz / 2;
      $("wf-lo").textContent = lo.toFixed(1); $("wf-ctr").textContent = m.center_mhz + " MHz"; $("wf-hi").textContent = hi.toFixed(1);
      $("rx-meta").textContent = `RMS ${m.rms.toFixed(1)} LSB · span ${m.span_mhz.toFixed(0)} MHz`;
    } else if (m.type === "rx_error") {
      $("rx-meta").textContent = "RX error: " + m.error;
    } else if (m.type === "link_progress") {
      $("lk-status").textContent = `Transmitting… ${m.done}/${m.total} bits (${Math.round(100 * m.done / m.total)}%)`;
    } else if (m.type === "link") {
      linkResult(m);
    }
  };
}
function setStatus(t, ok) {
  const s = $("status"); s.textContent = t;
  s.className = "pill" + (ok ? " ok" : ok === false ? " bad" : "");
}

// ---- message link (single sender on TX pane; result shown under RX waterfall) ----
async function showTxPreview() {
  const p = await api("/api/link/preview", {message: $("lk-msg").value, modulation: $("lk-mod").value});
  if (p && !p.error) {
    drawWave($("tx-wave").getContext("2d"), $("tx-wave"), p.waveform, "#d29922");
    $("tx-meta").textContent = `${p.scheme} · ${p.bits.length} bits`;
  } else if (p && p.error) {
    $("tx-meta").textContent = p.error;
  }
}
$("lk-send").onclick = async () => {
  const btn = $("lk-send"), st = $("lk-status");
  btn.disabled = true; st.textContent = "Starting transmission…";
  $("lk-result").style.display = "none";
  showTxPreview();                       // show the generated waveform immediately
  const r = await api("/api/link/send", {
    message: $("lk-msg").value, modulation: $("lk-mod").value, avg: parseInt($("lk-avg").value) || 3,
  });
  if (!r || r.error) { st.textContent = "⚠ " + (r ? r.error : "request failed"); btn.disabled = false; }
  // success arrives asynchronously via the WS "link" message
};
$("lk-mod").onchange = showTxPreview;
$("lk-msg").oninput = () => { clearTimeout(window._pvt); window._pvt = setTimeout(showTxPreview, 250); };
function linkResult(r) {
  const st = $("lk-status");
  $("lk-send").disabled = false;
  if (r.error) { st.textContent = "⚠ " + r.error; return; }
  st.textContent = `${(r.info && r.info.scheme) || ""} · ${r.total_bits} bits · ${r.bit_errors} errors`;
  $("lk-result").style.display = "block";
  $("lk-sent").textContent = JSON.stringify(r.sent);
  $("lk-dec").textContent = JSON.stringify(r.decoded);
  $("lk-dec").style.color = r.matched ? "var(--green)" : (r.crc_ok ? "var(--ink)" : "var(--amber)");
  $("lk-ber").textContent = r.ber_pct + "%";
  $("lk-crc").textContent = r.crc_ok ? "✓ valid" : "✗";
  $("lk-crc").style.color = r.crc_ok ? "var(--green)" : "var(--danger)";
  $("lk-bits").textContent = `sent: ${r.sent_bits}\nrecv: ${r.recv_bits}`;
}

// ---- wire up ----
$("refresh").onclick = refresh;
$("rx-apply").onclick = () => api("/api/rx/config", {
  serial: $("rx-dev").value, freq: parseInt($("rx-freq").value), rate: parseInt($("rx-rate").value),
});
$("rx-dev").onchange = () => { if ($("rx-dev").value) $("rx-apply").onclick(); };

connect();
refresh();
showTxPreview();
setInterval(refresh, 5000);

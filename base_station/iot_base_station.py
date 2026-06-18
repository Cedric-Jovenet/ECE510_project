#!/usr/bin/env python3
import argparse
import json
import math
import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ECE510 Safety Dashboard</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Arial, Helvetica, sans-serif;
      --bg: #f4f6f8;
      --panel: #ffffff;
      --line: #d8dee6;
      --text: #17202a;
      --muted: #697586;
      --clear: #0a7f4f;
      --warning: #a56500;
      --critical: #bd1e35;
      --unknown: #667085;
    }
    * { box-sizing: border-box; }
    body { margin: 0; background: var(--bg); color: var(--text); }
    header {
      min-height: 58px;
      padding: 12px 18px;
      border-bottom: 1px solid var(--line);
      background: #ffffff;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 16px;
      position: sticky;
      top: 0;
      z-index: 10;
    }
    h1 { font-size: 20px; margin: 0; }
    h2 { font-size: 15px; margin: 0 0 10px; }
    h3 { font-size: 13px; margin: 0 0 8px; color: #344054; }
    main {
      display: grid;
      grid-template-columns: minmax(340px, 0.8fr) minmax(460px, 1.2fr);
      gap: 14px;
      padding: 14px;
      align-items: start;
    }
    section, details.report {
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 7px;
    }
    section { padding: 12px; }
    .summary {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 10px;
      margin-bottom: 14px;
    }
    .metric {
      background: #f8fafc;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 10px;
      min-height: 70px;
    }
    .metric span { color: var(--muted); font-size: 12px; display: block; }
    .metric b { font-size: 22px; margin-top: 7px; display: block; overflow-wrap: anywhere; }
    .devices, .reports { display: grid; gap: 10px; }
    details.device {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #ffffff;
    }
    details.device > summary,
    details.report > summary {
      cursor: pointer;
      list-style: none;
      padding: 12px;
      display: grid;
      grid-template-columns: auto 1fr auto;
      gap: 10px;
      align-items: center;
    }
    details.device > summary::-webkit-details-marker,
    details.report > summary::-webkit-details-marker { display: none; }
    .device-body, .report-body {
      border-top: 1px solid var(--line);
      padding: 12px;
      display: grid;
      gap: 12px;
    }
    .main-line { min-width: 0; }
    .main-line b { display: block; overflow-wrap: anywhere; }
    .small { color: var(--muted); font-size: 12px; }
    .pill {
      display: inline-flex;
      min-width: 74px;
      justify-content: center;
      border-radius: 999px;
      padding: 4px 9px;
      color: #ffffff;
      font-size: 12px;
      font-weight: 700;
      text-transform: uppercase;
    }
    .clear { background: var(--clear); }
    .warning { background: var(--warning); }
    .critical { background: var(--critical); }
    .unknown { background: var(--unknown); }
    .sources, .evidence-meta {
      display: flex;
      flex-wrap: wrap;
      gap: 6px;
      align-items: center;
    }
    .source-chip, .meta-chip {
      background: #eef2f6;
      color: #344054;
      border-radius: 999px;
      padding: 3px 8px;
      font-size: 12px;
    }
    .evidence-grid {
      display: grid;
      grid-template-columns: minmax(280px, 1fr) minmax(280px, 1fr);
      gap: 12px;
    }
    .evidence-panel {
      min-width: 0;
      display: grid;
      gap: 8px;
      align-content: start;
    }
    .clip {
      width: 100%;
      max-height: 360px;
      object-fit: contain;
      background: #0b0f14;
      border: 1px solid var(--line);
      border-radius: 6px;
    }
    canvas.uwb-map {
      width: 100%;
      aspect-ratio: 1 / 0.72;
      display: block;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fbfcfe;
    }
    .empty {
      color: var(--muted);
      padding: 18px 8px;
      text-align: center;
      border: 1px dashed var(--line);
      border-radius: 6px;
    }
    details.subdetails {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fbfcfe;
    }
    details.subdetails summary {
      cursor: pointer;
      padding: 8px 10px;
      color: #344054;
      font-weight: 700;
    }
    pre {
      margin: 0;
      padding: 10px;
      max-height: 260px;
      overflow: auto;
      white-space: pre-wrap;
      word-break: break-word;
      color: #182230;
      font-size: 12px;
      line-height: 1.35;
      border-top: 1px solid var(--line);
    }
    a { color: #075985; }
    @media (max-width: 980px) {
      main, .evidence-grid { grid-template-columns: 1fr; }
      .summary { grid-template-columns: repeat(2, 1fr); }
    }
  </style>
</head>
<body>
  <header>
    <h1>ECE510 Safety Dashboard</h1>
    <span id="clock" class="small">loading...</span>
  </header>
  <main>
    <section>
      <h2>System Status</h2>
      <div class="summary">
        <div class="metric"><span>Active Devices</span><b id="metric-devices">0</b></div>
        <div class="metric"><span>Highest Level</span><b id="metric-level">unknown</b></div>
        <div class="metric"><span>Accident Reports</span><b id="metric-reports">0</b></div>
        <div class="metric"><span>LoRa Packets</span><b id="metric-lora">0</b></div>
      </div>
      <div id="devices" class="devices"></div>
      <details class="subdetails" style="margin-top:12px">
        <summary>LoRa and API Raw State</summary>
        <pre id="raw">waiting...</pre>
      </details>
    </section>
    <section>
      <h2>Accident Reports</h2>
      <div id="reports" class="reports"></div>
    </section>
  </main>
<script>
const DEFAULT_UWB = {
  anchors: [{id: 1, x: -0.35, y: 0}, {id: 2, x: 0.35, y: 0}, {id: 3, x: 0, y: 0.55}],
  zones: {critical_radius_m: 1.5, warning_radius_m: 3.0},
  workers: []
};
let renderedReportsSignature = null;

function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>"']/g, c => ({
    '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
  }[c]));
}
function levelClass(level) {
  return ['clear','warning','critical'].includes(level) ? level : 'unknown';
}
function levelRank(level) {
  return {unknown: 0, clear: 1, warning: 2, critical: 3}[level] || 0;
}
function fmtAge(seconds) {
  if (seconds == null || !Number.isFinite(Number(seconds))) return 'n/a';
  const value = Number(seconds);
  if (value < 2) return `${value.toFixed(1)}s`;
  return `${Math.round(value)}s`;
}
function fmtTime(seconds) {
  return seconds ? new Date(Number(seconds) * 1000).toLocaleString() : 'n/a';
}
function payloadOf(report) {
  return report?.payload || report || {};
}
function mediaOf(report) {
  const payload = payloadOf(report);
  return payload.media || payload.status?.media || {};
}
function evidenceOf(report) {
  return payloadOf(report).evidence || {};
}
function statusOfReport(report) {
  const payload = payloadOf(report);
  return payload.status || payload;
}
function reportKey(report, index) {
  const payload = payloadOf(report);
  return payload.report_id || `${report.device_id || payload.device_id || 'unknown'}-${report.time || payload.time || index}`;
}
function sourcesOf(device) {
  return device?.payload?.active_sources || {};
}
function sourceSummary(device) {
  const items = Object.entries(sourcesOf(device));
  if (!items.length) return '<span class="small">No detailed source yet</span>';
  return items.map(([name, source]) =>
    `<span class="source-chip">${escapeHtml(name)}: ${escapeHtml(source.level || 'unknown')}</span>`
  ).join('');
}
function normalizeUwb(data) {
  const raw = data && typeof data === 'object' ? data : {};
  return {
    time: Number(raw.time) || undefined,
    anchors: Array.isArray(raw.anchors) && raw.anchors.length ? raw.anchors : DEFAULT_UWB.anchors,
    zones: raw.zones || DEFAULT_UWB.zones,
    workers: Array.isArray(raw.workers) ? raw.workers : []
  };
}
function evidenceHistory(report) {
  const evidence = evidenceOf(report);
  if (Array.isArray(evidence.uwb_history) && evidence.uwb_history.length) {
    return evidence.uwb_history.map(normalizeUwb);
  }
  if (evidence.uwb_snapshot && Object.keys(evidence.uwb_snapshot).length) {
    return [normalizeUwb(evidence.uwb_snapshot)];
  }
  return [];
}
function evidenceSummary(report) {
  const evidence = evidenceOf(report);
  const history = evidenceHistory(report);
  const workerSamples = history.reduce((sum, sample) => sum + (sample.workers || []).length, 0);
  const window = evidence.window || {};
  const before = Number(window.before_sec);
  const after = Number(window.after_sec);
  const chips = [
    `<span class="meta-chip">${history.length} UWB samples</span>`,
    `<span class="meta-chip">${workerSamples} worker positions</span>`
  ];
  if (Number.isFinite(before) && Number.isFinite(after)) {
    chips.push(`<span class="meta-chip">${before.toFixed(1)}s before / ${after.toFixed(1)}s after</span>`);
  }
  return chips.join('');
}
function sourceText(status) {
  const sources = status?.active_sources || {};
  return Object.entries(sources).map(([name, source]) =>
    `${name}:${source.level || 'unknown'}`
  ).join('  ') || 'sources n/a';
}
function worldToCanvas(x, y, scale, cx, cy) {
  return [cx + x * scale, cy - y * scale];
}
function drawCircle(ctx, x, y, radius, stroke, fill, lineWidth = 2) {
  ctx.beginPath();
  ctx.arc(x, y, Math.max(0, radius), 0, Math.PI * 2);
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  ctx.strokeStyle = stroke;
  ctx.lineWidth = lineWidth;
  ctx.stroke();
}
function workerColor(level) {
  if (level === 'critical') return '#bd1e35';
  if (level === 'warning') return '#a56500';
  return '#0a7f4f';
}
function drawUwbMovement(canvas, history) {
  const ctx = canvas.getContext('2d');
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#fbfcfe';
  ctx.fillRect(0, 0, width, height);

  const normalized = history.map(normalizeUwb);
  const reference = normalized.find(sample => sample.anchors?.length) || normalizeUwb(DEFAULT_UWB);
  const anchors = reference.anchors || DEFAULT_UWB.anchors;
  const zones = reference.zones || DEFAULT_UWB.zones;
  const tracks = new Map();
  let maxExtent = Math.max(2.0, Number(zones.warning_radius_m) || 3.0);

  for (const anchor of anchors) {
    maxExtent = Math.max(maxExtent, Math.abs(Number(anchor.x) || 0), Math.abs(Number(anchor.y) || 0));
  }
  for (const sample of normalized) {
    for (const worker of sample.workers || []) {
      const x = Number(worker.x);
      const y = Number(worker.y);
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      const id = String(worker.id || worker.worker_id || 'worker');
      if (!tracks.has(id)) tracks.set(id, []);
      tracks.get(id).push({
        x, y,
        time: Number(sample.time) || 0,
        level: worker.warning_level || 'clear',
        distance: Number(worker.distance_to_machine_m)
      });
      maxExtent = Math.max(maxExtent, Math.abs(x), Math.abs(y));
    }
  }

  const scale = (Math.min(width, height) * 0.40) / Math.max(1.0, maxExtent);
  const cx = width * 0.5;
  const cy = height * 0.57;

  ctx.strokeStyle = '#e4e7ec';
  ctx.lineWidth = 1;
  for (let g = -Math.ceil(maxExtent); g <= Math.ceil(maxExtent); g++) {
    const [x] = worldToCanvas(g, 0, scale, cx, cy);
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, height); ctx.stroke();
    const [, y] = worldToCanvas(0, g, scale, cx, cy);
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke();
  }

  const warning = Number(zones.warning_radius_m) || 3.0;
  const critical = Number(zones.critical_radius_m) || 1.5;
  drawCircle(ctx, cx, cy, warning * scale, '#d39b2a', 'rgba(211,155,42,0.09)', 2);
  drawCircle(ctx, cx, cy, critical * scale, '#c43f4d', 'rgba(196,63,77,0.11)', 2);

  ctx.fillStyle = '#182230';
  ctx.fillRect(cx - 18, cy - 24, 36, 48);
  ctx.fillStyle = '#ffffff';
  ctx.font = '14px Arial';
  ctx.fillText('machine', cx + 25, cy + 5);

  ctx.font = '14px Arial';
  for (const anchor of anchors) {
    const [x, y] = worldToCanvas(Number(anchor.x) || 0, Number(anchor.y) || 0, scale, cx, cy);
    ctx.fillStyle = '#008b8b';
    ctx.beginPath();
    ctx.moveTo(x, y - 10);
    ctx.lineTo(x + 10, y + 9);
    ctx.lineTo(x - 10, y + 9);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = '#155e63';
    ctx.fillText(`A${anchor.id}`, x + 12, y - 8);
  }

  for (const [id, points] of tracks.entries()) {
    if (!points.length) continue;
    points.sort((a, b) => a.time - b.time);
    const last = points[points.length - 1];
    const color = workerColor(last.level);
    ctx.strokeStyle = color;
    ctx.lineWidth = 3;
    ctx.beginPath();
    points.forEach((point, index) => {
      const [x, y] = worldToCanvas(point.x, point.y, scale, cx, cy);
      if (index === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
    points.forEach((point, index) => {
      const [x, y] = worldToCanvas(point.x, point.y, scale, cx, cy);
      const alpha = 0.35 + 0.65 * ((index + 1) / points.length);
      drawCircle(ctx, x, y, index === points.length - 1 ? 11 : 5, color, `rgba(23,32,42,${alpha})`, 1);
    });
    const [x, y] = worldToCanvas(last.x, last.y, scale, cx, cy);
    ctx.fillStyle = '#182230';
    const distance = Number.isFinite(last.distance) ? ` ${last.distance.toFixed(2)} m` : '';
    ctx.fillText(`${id}${distance}`, x + 16, y - 10);
  }

  if (!tracks.size) {
    ctx.fillStyle = '#697586';
    ctx.font = '16px Arial';
    ctx.fillText('No UWB movement recorded in this window', 22, 32);
  }
}
function renderDevices(state) {
  const root = document.getElementById('devices');
  const open = new Set([...root.querySelectorAll('details.device[open]')].map(el => el.dataset.key));
  const devices = Object.values(state.devices || {}).sort((a, b) => String(a.device_id).localeCompare(String(b.device_id)));
  if (!devices.length) {
    root.innerHTML = '<div class="empty">No device ping received yet</div>';
    return;
  }
  root.innerHTML = devices.map(device => {
    const key = String(device.device_id || 'unknown');
    return `
      <details class="device" data-key="${escapeHtml(key)}" ${open.has(key) ? 'open' : ''}>
        <summary>
          <span class="pill ${levelClass(device.level)}">${escapeHtml(device.level || 'unknown')}</span>
          <span class="main-line"><b>${escapeHtml(key)}</b><span class="small">${escapeHtml(device.device_type || '')} | ${escapeHtml(device.transport || '')} | ${fmtAge(device.age_sec)}</span></span>
          <span class="small">details</span>
        </summary>
        <div class="device-body">
          <div class="sources">${sourceSummary(device)}</div>
          <details class="subdetails">
            <summary>Raw Payload</summary>
            <pre>${escapeHtml(JSON.stringify(device.payload || device, null, 2))}</pre>
          </details>
        </div>
      </details>`;
  }).join('');
}
function renderReports(state) {
  const root = document.getElementById('reports');
  const open = new Set([...root.querySelectorAll('details.report[open]')].map(el => el.dataset.key));
  const reports = (state.reports || []).slice().reverse();
  const signature = reports.map((report, index) => {
    const payload = payloadOf(report);
    const media = mediaOf(report);
    const evidence = evidenceOf(report);
    const history = Array.isArray(evidence.uwb_history) ? evidence.uwb_history.length : 0;
    return `${reportKey(report, index)}:${media.recorded === true}:${history}`;
  }).join('|');
  if (signature === renderedReportsSignature) return;
  renderedReportsSignature = signature;
  if (!reports.length) {
    root.innerHTML = '<div class="empty">No accident reports yet</div>';
    return;
  }
  root.innerHTML = reports.map((report, index) => {
    const payload = payloadOf(report);
    const status = statusOfReport(report);
    const media = mediaOf(report);
    const key = reportKey(report, index);
    const hasRecordedClip = Boolean(media.clip_url && media.recorded === true);
    const clipUrl = hasRecordedClip ? media.clip_url : '';
    const clip = hasRecordedClip
      ? `<img class="clip" src="${escapeHtml(clipUrl)}" alt="Accident evidence video">`
      : '<div class="empty">No recorded video evidence for this report</div>';
    return `
      <details class="report" data-key="${escapeHtml(key)}" ${open.has(key) ? 'open' : ''}>
        <summary>
          <span class="pill ${levelClass(report.level)}">${escapeHtml(report.level || 'report')}</span>
          <span class="main-line"><b>${escapeHtml(report.device_id || payload.device_id || 'unknown')}</b><span class="small">${fmtTime(report.time || payload.time)} | ${escapeHtml(report.transport || '')} | ${escapeHtml(sourceText(status))}</span></span>
          <span class="small">details</span>
        </summary>
        <div class="report-body">
          <div class="evidence-meta">${evidenceSummary(report)}</div>
          <div class="evidence-grid">
            <div class="evidence-panel">
              <h3>Evidence Video</h3>
              ${clip}
              ${media.viewer_url ? `<div class="small"><a href="${escapeHtml(media.viewer_url)}" target="_blank" rel="noreferrer">Open machine viewer</a></div>` : ''}
            </div>
            <div class="evidence-panel">
              <h3>UWB Movement Around Accident</h3>
              <canvas class="uwb-map report-map" width="800" height="560" data-report-index="${index}"></canvas>
            </div>
          </div>
          <details class="subdetails">
            <summary>Raw Accident Data</summary>
            <pre>${escapeHtml(JSON.stringify(payload || report, null, 2))}</pre>
          </details>
        </div>
      </details>`;
  }).join('');
  for (const canvas of root.querySelectorAll('canvas.report-map')) {
    const report = reports[Number(canvas.dataset.reportIndex)];
    drawUwbMovement(canvas, evidenceHistory(report));
  }
}
function refreshMetrics(state) {
  const devices = Object.values(state.devices || {});
  let maxLevel = 'unknown';
  for (const device of devices) if (levelRank(device.level) > levelRank(maxLevel)) maxLevel = device.level;
  document.getElementById('metric-devices').textContent = String(devices.length);
  document.getElementById('metric-level').textContent = maxLevel;
  document.getElementById('metric-reports').textContent = String((state.reports || []).length);
  document.getElementById('metric-lora').textContent = String(state.lora?.packet_count || 0);
  document.getElementById('clock').textContent = new Date().toLocaleTimeString();
}
async function refresh() {
  const res = await fetch('/api/state', {cache: 'no-store'});
  const state = await res.json();
  refreshMetrics(state);
  renderDevices(state);
  renderReports(state);
  document.getElementById('raw').textContent = JSON.stringify({lora: state.lora, devices: state.devices}, null, 2);
}
setInterval(refresh, 1000);
refresh().catch(err => {
  document.getElementById('reports').innerHTML = `<div class="empty">${escapeHtml(err)}</div>`;
});
</script>
</body>
</html>
"""


class StateStore:
    def __init__(self, max_reports=100):
        self.lock = threading.Lock()
        self.devices = {}
        self.reports = []
        self.max_reports = max_reports
        self.lora = {
            'enabled': False,
            'ok': False,
            'last_packet': None,
            'last_error': None,
            'packet_count': 0,
        }

    def update_ping(self, payload, transport):
        now = time.time()
        device_id = str(payload.get('device_id') or payload.get('worker_id') or 'unknown')
        device = {
            'device_id': device_id,
            'device_type': payload.get('device_type') or payload.get('type') or 'unknown',
            'level': payload.get('level') or payload.get('warning_level') or 'unknown',
            'time': float(payload.get('time', now)),
            'age_sec': 0.0,
            'transport': transport,
            'payload': payload,
        }
        with self.lock:
            self.devices[device_id] = device

    def add_report(self, payload, transport):
        now = time.time()
        report = {
            'report_id': payload.get('report_id'),
            'time': float(payload.get('time', now)),
            'device_id': payload.get('device_id') or payload.get('worker_id') or 'unknown',
            'device_type': payload.get('device_type') or payload.get('type') or 'unknown',
            'level': payload.get('level') or 'critical',
            'transport': transport,
            'payload': payload,
        }
        with self.lock:
            self.reports.append(report)
            self.reports = self.reports[-self.max_reports:]

    def update_lora(self, **kwargs):
        with self.lock:
            self.lora.update(kwargs)

    def snapshot(self):
        now = time.time()
        with self.lock:
            devices = {}
            for device_id, device in self.devices.items():
                item = dict(device)
                item['age_sec'] = max(0.0, now - float(item.get('time', now)))
                devices[device_id] = item
            return {
                'time': now,
                'hostname': socket.gethostname(),
                'devices': devices,
                'reports': list(self.reports),
                'lora': dict(self.lora),
            }


class ApiHandler(BaseHTTPRequestHandler):
    store = None

    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ('/', '/index.html'):
            self._send(200, HTML.encode('utf-8'), 'text/html; charset=utf-8')
        elif path == '/api/state':
            data = json.dumps(self.store.snapshot()).encode('utf-8')
            self._send(200, data, 'application/json')
        else:
            self.send_error(404)

    def do_POST(self):
        path = urlsplit(self.path).path
        length = int(self.headers.get('Content-Length', '0') or '0')
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode('utf-8'))
        except json.JSONDecodeError:
            self.send_error(400, 'Invalid JSON')
            return

        if path == '/api/ping':
            self.store.update_ping(payload, 'http')
            self._send(200, b'{"ok":true}', 'application/json')
        elif path == '/api/report':
            self.store.add_report(payload, 'http')
            self._send(200, b'{"ok":true}', 'application/json')
        else:
            self.send_error(404)

    def _send(self, status, data, content_type):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


class Rfm95Receiver(threading.Thread):
    REG_FIFO = 0x00
    REG_OP_MODE = 0x01
    REG_FRF_MSB = 0x06
    REG_PA_CONFIG = 0x09
    REG_LNA = 0x0C
    REG_FIFO_ADDR_PTR = 0x0D
    REG_FIFO_RX_BASE_ADDR = 0x0F
    REG_FIFO_RX_CURRENT_ADDR = 0x10
    REG_IRQ_FLAGS = 0x12
    REG_RX_NB_BYTES = 0x13
    REG_MODEM_CONFIG_1 = 0x1D
    REG_MODEM_CONFIG_2 = 0x1E
    REG_PREAMBLE_MSB = 0x20
    REG_PREAMBLE_LSB = 0x21
    REG_SYNC_WORD = 0x39
    REG_DIO_MAPPING_1 = 0x40
    REG_VERSION = 0x42

    IRQ_RX_DONE = 0x40
    IRQ_PAYLOAD_CRC_ERROR = 0x20

    def __init__(self, store, spi_bus=0, spi_device=0, reset_gpio=25,
                 gpiochip=4, frequency_mhz=915.0):
        super().__init__(daemon=True)
        self.store = store
        self.spi_bus = spi_bus
        self.spi_device = spi_device
        self.reset_gpio = reset_gpio
        self.gpiochip = gpiochip
        self.frequency_mhz = frequency_mhz
        self.stop_event = threading.Event()
        self.spi = None

    def run(self):
        try:
            import spidev
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = 500000
            self.spi.mode = 0
            self._reset()
            version = self._read(self.REG_VERSION)
            if version != 0x12:
                raise RuntimeError(f'RFM95 not detected, RegVersion=0x{version:02x}')
            self._configure()
            self.store.update_lora(enabled=True, ok=True, version='0x12')
            while not self.stop_event.is_set():
                self._poll_once()
                time.sleep(0.02)
        except Exception as exc:
            self.store.update_lora(enabled=True, ok=False, last_error=str(exc))
        finally:
            if self.spi is not None:
                self.spi.close()

    def stop(self):
        self.stop_event.set()

    def _reset(self):
        try:
            import lgpio
            handle = lgpio.gpiochip_open(self.gpiochip)
            try:
                lgpio.gpio_claim_output(handle, self.reset_gpio, 1)
                time.sleep(0.05)
                lgpio.gpio_write(handle, self.reset_gpio, 0)
                time.sleep(0.1)
                lgpio.gpio_write(handle, self.reset_gpio, 1)
                time.sleep(0.2)
            finally:
                try:
                    lgpio.gpio_free(handle, self.reset_gpio)
                except Exception:
                    pass
                lgpio.gpiochip_close(handle)
        except Exception:
            time.sleep(0.2)

    def _read(self, address):
        return self.spi.xfer2([address & 0x7F, 0x00])[1]

    def _write(self, address, value):
        self.spi.xfer2([address | 0x80, value & 0xFF])

    def _configure(self):
        self._write(self.REG_OP_MODE, 0x80)  # LoRa sleep
        time.sleep(0.01)
        frf = int((self.frequency_mhz * 1_000_000.0) / 61.03515625)
        self._write(self.REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._write(self.REG_FRF_MSB + 1, (frf >> 8) & 0xFF)
        self._write(self.REG_FRF_MSB + 2, frf & 0xFF)
        self._write(self.REG_FIFO_RX_BASE_ADDR, 0)
        self._write(self.REG_FIFO_ADDR_PTR, 0)
        self._write(self.REG_LNA, self._read(self.REG_LNA) | 0x03)
        self._write(self.REG_MODEM_CONFIG_1, 0x72)  # BW125, CR4/5, explicit header
        self._write(self.REG_MODEM_CONFIG_2, 0x74)  # SF7, CRC on
        self._write(self.REG_PREAMBLE_MSB, 0x00)
        self._write(self.REG_PREAMBLE_LSB, 0x08)
        self._write(self.REG_SYNC_WORD, 0x34)
        self._write(self.REG_DIO_MAPPING_1, 0x00)
        self._write(self.REG_IRQ_FLAGS, 0xFF)
        self._write(self.REG_OP_MODE, 0x85)  # LoRa RX continuous

    def _poll_once(self):
        flags = self._read(self.REG_IRQ_FLAGS)
        if not flags:
            return
        self._write(self.REG_IRQ_FLAGS, flags)
        if not (flags & self.IRQ_RX_DONE):
            return
        if flags & self.IRQ_PAYLOAD_CRC_ERROR:
            self.store.update_lora(last_error='crc_error')
            return

        length = self._read(self.REG_RX_NB_BYTES)
        current_addr = self._read(self.REG_FIFO_RX_CURRENT_ADDR)
        self._write(self.REG_FIFO_ADDR_PTR, current_addr)
        data = bytes(self._read(self.REG_FIFO) for _ in range(length))
        text = data.decode('utf-8', errors='replace').strip()
        packet = {'time': time.time(), 'text': text, 'bytes': length}
        self.store.update_lora(
            last_packet=packet,
            packet_count=self.store.snapshot()['lora'].get('packet_count', 0) + 1,
            last_error=None,
        )
        self._ingest_packet(text)

    def _ingest_packet(self, text):
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return
        kind = str(payload.get('type', '')).lower()
        if 'report' in kind:
            self.store.add_report(payload, 'lora')
        else:
            self.store.update_ping(payload, 'lora')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    parser.add_argument('--no-lora', action='store_true')
    parser.add_argument('--frequency-mhz', type=float, default=915.0)
    args = parser.parse_args()

    store = StateStore()
    receiver = None
    if not args.no_lora:
        receiver = Rfm95Receiver(store, frequency_mhz=args.frequency_mhz)
        receiver.start()
    else:
        store.update_lora(enabled=False, ok=False)

    ApiHandler.store = store
    server = ReusableThreadingHTTPServer((args.host, args.port), ApiHandler)
    server.timeout = 0.5
    stopping = {'value': False}

    def stop(signum, frame):
        del signum, frame
        stopping['value'] = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    print(f'Base station dashboard: http://{socket.gethostname()}:{args.port}/', flush=True)
    try:
        while not stopping['value']:
            server.handle_request()
    finally:
        if receiver is not None:
            receiver.stop()
        server.server_close()


if __name__ == '__main__':
    main()

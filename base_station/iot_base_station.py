#!/usr/bin/env python3
# Lightweight HTTP dashboard and optional RFM95 LoRa receiver for the ECE510
# machine/worker safety demo. It stores only recent status and accident reports
# in memory so the base can run as a simple field gateway.
import argparse
import json
import math
import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


# The dashboard is embedded so the base station can run as one Python file on a
# Raspberry Pi or in `--no-lora` cloud-mirror mode without a static asset step.
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
    .clip-viewer {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #0b0f14;
      overflow: hidden;
    }
    .clip-frame {
      width: 100%;
      max-height: 360px;
      object-fit: contain;
      display: block;
      background: #0b0f14;
    }
    .clip-controls {
      display: grid;
      grid-template-columns: auto minmax(80px, 1fr) auto;
      gap: 8px;
      align-items: center;
      padding: 8px;
      background: #ffffff;
      border-top: 1px solid var(--line);
    }
    .clip-controls button {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #f8fafc;
      color: var(--text);
      cursor: pointer;
      font-weight: 700;
      padding: 6px 10px;
    }
    .clip-controls input[type="range"] { width: 100%; }
    canvas.uwb-map {
      width: 100%;
      aspect-ratio: 1 / 0.72;
      display: block;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #fbfcfe;
    }
    .uwb-player {
      border: 1px solid var(--line);
      border-radius: 6px;
      overflow: hidden;
      background: #fbfcfe;
    }
    .uwb-player canvas.uwb-map {
      border: 0;
      border-radius: 0;
    }
    .uwb-controls {
      display: grid;
      grid-template-columns: auto minmax(80px, 1fr) auto auto;
      gap: 8px;
      align-items: center;
      padding: 8px;
      background: #ffffff;
      border-top: 1px solid var(--line);
    }
    .uwb-controls button {
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #f8fafc;
      color: var(--text);
      cursor: pointer;
      font-weight: 700;
      padding: 6px 10px;
    }
    .uwb-controls input[type="range"] { width: 100%; }
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
  zones: {critical_radius_m: 2.0, warning_radius_m: 2.0},
  workers: []
};
let renderedReportsSignature = null;
const clipTimers = new Map();
const uwbTimers = new Map();
let refreshInFlight = false;

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
function frameUrlFromTemplate(template, index) {
  return String(template || '').replace('{index}', String(index));
}
function frameTemplateFromMedia(media) {
  if (media.frame_url_template) return media.frame_url_template;
  if (media.clip_url && media.clip_id && media.clip_url.includes('/recorded_clip.mjpg')) {
    const base = media.clip_url.split('/recorded_clip.mjpg')[0];
    return `${base}/recorded_frame.jpg?clip_id=${encodeURIComponent(media.clip_id)}&index={index}`;
  }
  return '';
}
function clipFrameCount(media) {
  return Number(media.frame_count || media.archive_result?.frame_count || 0);
}
function clipFps(media) {
  return Math.max(1, Math.min(12, Number(media.clip_fps || media.archive_result?.fps || 4)));
}
function buildClipViewer(media, key) {
  const frameCount = clipFrameCount(media);
  const template = frameTemplateFromMedia(media);
  if (!(media.recorded === true && frameCount > 0 && template)) {
    return '<div class="empty">No recorded video evidence for this report</div>';
  }
  const safeKey = escapeHtml(key);
  const safeTemplate = escapeHtml(template);
  const firstFrame = escapeHtml(frameUrlFromTemplate(template, 0));
  return `
    <div class="clip-viewer" data-clip-key="${safeKey}" data-frame-count="${frameCount}" data-fps="${clipFps(media)}" data-template="${safeTemplate}">
      <img class="clip-frame" data-role="frame" src="${firstFrame}" alt="Recorded accident frame">
      <div class="clip-controls">
        <button type="button" data-action="toggle">Play</button>
        <input type="range" data-role="slider" min="0" max="${Math.max(0, frameCount - 1)}" value="0" step="1" aria-label="Video frame">
        <span class="small" data-role="counter">1 / ${frameCount}</span>
      </div>
    </div>`;
}
function buildUwbAnimation(report, key, index) {
  const frameCount = evidenceHistory(report).length;
  if (frameCount <= 0) {
    return '<div class="empty">No UWB movement recorded in this window</div>';
  }
  const safeKey = escapeHtml(key);
  const maxFrame = Math.max(0, frameCount - 1);
  return `
    <div class="uwb-player" data-uwb-key="${safeKey}" data-report-index="${index}" data-frame-count="${frameCount}" data-fps="4">
      <canvas class="uwb-map report-map" width="800" height="560"></canvas>
      <div class="uwb-controls">
        <button type="button" data-action="toggle">Play</button>
        <input type="range" data-role="slider" min="0" max="${maxFrame}" value="0" step="1" aria-label="UWB frame">
        <span class="small" data-role="counter">1 / ${frameCount}</span>
        <span class="small" data-role="timestamp">time n/a</span>
      </div>
    </div>`;
}
function stopClipTimer(key) {
  const timer = clipTimers.get(key);
  if (timer) {
    clearInterval(timer);
    clipTimers.delete(key);
  }
}
function stopAllClipTimers() {
  for (const key of Array.from(clipTimers.keys())) stopClipTimer(key);
}
function stopUwbTimer(key) {
  const timer = uwbTimers.get(key);
  if (timer) {
    clearInterval(timer);
    uwbTimers.delete(key);
  }
}
function stopAllUwbTimers() {
  for (const key of Array.from(uwbTimers.keys())) stopUwbTimer(key);
}
function setClipFrame(player, index) {
  const frameCount = Number(player.dataset.frameCount || 0);
  const bounded = Math.max(0, Math.min(frameCount - 1, Number(index) || 0));
  const template = player.dataset.template || '';
  const image = player.querySelector('[data-role="frame"]');
  const slider = player.querySelector('[data-role="slider"]');
  const counter = player.querySelector('[data-role="counter"]');
  if (image) image.src = frameUrlFromTemplate(template, bounded);
  if (slider) slider.value = String(bounded);
  if (counter) counter.textContent = `${bounded + 1} / ${frameCount}`;
  player.dataset.currentFrame = String(bounded);
}
function startClip(player) {
  const key = player.dataset.clipKey;
  const frameCount = Number(player.dataset.frameCount || 0);
  const fps = Number(player.dataset.fps || 4);
  const button = player.querySelector('[data-action="toggle"]');
  if (!key || frameCount <= 1) return;
  stopClipTimer(key);
  if (Number(player.dataset.currentFrame || 0) >= frameCount - 1) setClipFrame(player, 0);
  if (button) button.textContent = 'Pause';
  const timer = setInterval(() => {
    const current = Number(player.dataset.currentFrame || 0);
    if (current >= frameCount - 1) {
      stopClipTimer(key);
      if (button) button.textContent = 'Play';
      return;
    }
    setClipFrame(player, current + 1);
  }, 1000 / fps);
  clipTimers.set(key, timer);
}
function initClipViewers(root) {
  for (const player of root.querySelectorAll('.clip-viewer')) {
    const key = player.dataset.clipKey;
    setClipFrame(player, Number(player.dataset.currentFrame || 0));
    const button = player.querySelector('[data-action="toggle"]');
    const slider = player.querySelector('[data-role="slider"]');
    if (button) {
      button.addEventListener('click', () => {
        if (clipTimers.has(key)) {
          stopClipTimer(key);
          button.textContent = 'Play';
        } else {
          startClip(player);
        }
      });
    }
    if (slider) {
      slider.addEventListener('input', () => {
        stopClipTimer(key);
        if (button) button.textContent = 'Play';
        setClipFrame(player, Number(slider.value));
      });
    }
  }
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
function finiteNumber(value) {
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}
function workerDisplayPosition(worker) {
  if (worker.display_position_valid === false) return null;
  const displayX = finiteNumber(worker.display_x);
  const displayY = finiteNumber(worker.display_y);
  if (displayX !== null && displayY !== null) return [displayX, displayY];
  const rawX = finiteNumber(worker.x);
  const rawY = finiteNumber(worker.y);
  if (rawX !== null && rawY !== null) return [rawX, rawY];
  return null;
}
function workerDistance(worker) {
  const displayDistance = finiteNumber(worker.display_distance_to_machine_m);
  if (displayDistance !== null) return displayDistance;
  return finiteNumber(worker.distance_to_machine_m);
}
function drawUwbFrame(canvas, history, frameIndex) {
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
  const frameCount = normalized.length;
  const boundedFrame = Math.max(0, Math.min(frameCount - 1, Number(frameIndex) || 0));
  const current = frameCount ? normalized[boundedFrame] : normalizeUwb(DEFAULT_UWB);
  let maxExtent = Math.max(2.0, Number(zones.warning_radius_m) || 3.0);

  for (const anchor of anchors) {
    maxExtent = Math.max(maxExtent, Math.abs(Number(anchor.x) || 0), Math.abs(Number(anchor.y) || 0));
  }
  for (const sample of normalized) {
    for (const worker of sample.workers || []) {
      const position = workerDisplayPosition(worker);
      if (!position) continue;
      const [x, y] = position;
      maxExtent = Math.max(maxExtent, Math.abs(x), Math.abs(y));
      for (const range of worker.ranges || []) {
        const anchor = anchors.find(item => String(item.id) === String(range.anchor_id));
        const distance = Number(range.distance_m);
        if (!anchor || !Number.isFinite(distance)) continue;
        maxExtent = Math.max(
          maxExtent,
          Math.abs(Number(anchor.x) || 0) + distance,
          Math.abs(Number(anchor.y) || 0) + distance
        );
      }
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

  let drawnWorkers = 0;
  for (const worker of current.workers || []) {
    const position = workerDisplayPosition(worker);
    if (!position) continue;
    drawnWorkers += 1;
    const [workerX, workerY] = position;
    const [x, y] = worldToCanvas(workerX, workerY, scale, cx, cy);
    const level = worker.warning_level || 'clear';
    const color = workerColor(level);
    const partial = worker.position_quality && worker.position_quality !== 'trilaterated';

    for (const range of worker.ranges || []) {
      const anchor = anchors.find(item => String(item.id) === String(range.anchor_id));
      const distance = Number(range.distance_m);
      if (!anchor || !Number.isFinite(distance)) continue;
      const [ax, ay] = worldToCanvas(Number(anchor.x) || 0, Number(anchor.y) || 0, scale, cx, cy);
      ctx.setLineDash([7, 7]);
      drawCircle(ctx, ax, ay, distance * scale, 'rgba(43,120,190,0.28)', null, partial ? 2 : 1);
      ctx.setLineDash([]);
      ctx.strokeStyle = 'rgba(102,112,133,0.24)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(x, y); ctx.stroke();
    }

    drawCircle(ctx, x, y, partial ? 10 : 12, color, partial ? 'rgba(255,255,255,0)' : color, 2);
    const velocity = worker.display_velocity || worker.velocity || {};
    const vx = Number(velocity.vx) || 0;
    const vy = Number(velocity.vy) || 0;
    const speed = Number(velocity.speed_mps) || 0;
    if (speed > 0.03) {
      const [ex, ey] = worldToCanvas(workerX + vx * 1.2, workerY + vy * 1.2, scale, cx, cy);
      ctx.strokeStyle = '#182230';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(ex, ey); ctx.stroke();
    }

    ctx.fillStyle = '#182230';
    ctx.font = '14px Arial';
    const id = String(worker.id || worker.worker_id || 'worker');
    const distance = workerDistance(worker);
    const distanceText = distance !== null ? ` ${distance.toFixed(2)} m` : '';
    const modeText = worker.display_mode && worker.display_mode !== 'uwb' ? ` ${worker.display_mode}` : '';
    ctx.fillText(`${id}${distanceText}${modeText}`, x + 16, y - 10);
  }

  if (!drawnWorkers) {
    ctx.fillStyle = '#697586';
    ctx.font = '16px Arial';
    ctx.fillText('No UWB worker in this frame', 22, 32);
  }

  ctx.fillStyle = '#344054';
  ctx.font = '13px Arial';
  const timestamp = current.time ? new Date(Number(current.time) * 1000).toLocaleTimeString() : 'time n/a';
  ctx.fillText(`UWB frame ${frameCount ? boundedFrame + 1 : 0} / ${frameCount} - ${timestamp}`, 22, height - 18);
}
function setUwbFrame(player, history, index) {
  const frameCount = history.length;
  const bounded = frameCount ? Math.max(0, Math.min(frameCount - 1, Number(index) || 0)) : 0;
  const canvas = player.querySelector('canvas.report-map');
  const slider = player.querySelector('[data-role="slider"]');
  const counter = player.querySelector('[data-role="counter"]');
  const timestamp = player.querySelector('[data-role="timestamp"]');
  if (canvas) drawUwbFrame(canvas, history, bounded);
  if (slider) slider.value = String(bounded);
  if (counter) counter.textContent = `${frameCount ? bounded + 1 : 0} / ${frameCount}`;
  const frameTime = history[bounded]?.time;
  if (timestamp) timestamp.textContent = frameTime ? new Date(Number(frameTime) * 1000).toLocaleTimeString() : 'time n/a';
  player.dataset.currentFrame = String(bounded);
}
function startUwbAnimation(player, history) {
  const key = player.dataset.uwbKey;
  const frameCount = history.length;
  const fps = Math.max(1, Math.min(12, Number(player.dataset.fps || 4)));
  const button = player.querySelector('[data-action="toggle"]');
  if (!key || frameCount <= 1) return;
  stopUwbTimer(key);
  if (button) button.textContent = 'Pause';
  const timer = setInterval(() => {
    const current = Number(player.dataset.currentFrame || 0);
    const next = current >= frameCount - 1 ? 0 : current + 1;
    setUwbFrame(player, history, next);
  }, 1000 / fps);
  uwbTimers.set(key, timer);
}
function initUwbAnimations(root, reports) {
  for (const player of root.querySelectorAll('.uwb-player')) {
    const key = player.dataset.uwbKey;
    const report = reports[Number(player.dataset.reportIndex)];
    const history = evidenceHistory(report);
    setUwbFrame(player, history, Number(player.dataset.currentFrame || 0));
    const button = player.querySelector('[data-action="toggle"]');
    const slider = player.querySelector('[data-role="slider"]');
    if (button) {
      button.disabled = history.length <= 1;
      button.addEventListener('click', () => {
        if (uwbTimers.has(key)) {
          stopUwbTimer(key);
          button.textContent = 'Play';
        } else {
          startUwbAnimation(player, history);
        }
      });
    }
    if (slider) {
      slider.disabled = history.length <= 1;
      slider.addEventListener('input', () => {
        stopUwbTimer(key);
        if (button) button.textContent = 'Play';
        setUwbFrame(player, history, Number(slider.value));
      });
    }
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
  const openRaw = new Set([...root.querySelectorAll('details.raw-details[open]')].map(el => el.dataset.rawKey));
  const reports = (state.reports || []).slice().reverse();
  const signature = reports.map((report, index) => {
    const payload = payloadOf(report);
    const media = mediaOf(report);
    const evidence = evidenceOf(report);
    const history = Array.isArray(evidence.uwb_history) ? evidence.uwb_history.length : 0;
    return `${reportKey(report, index)}:${media.recorded === true}:${clipFrameCount(media)}:${Boolean(frameTemplateFromMedia(media))}:${history}`;
  }).join('|');
  if (signature === renderedReportsSignature) return;
  renderedReportsSignature = signature;
  stopAllClipTimers();
  stopAllUwbTimers();
  if (!reports.length) {
    root.innerHTML = '<div class="empty">No accident reports yet</div>';
    return;
  }
  root.innerHTML = reports.map((report, index) => {
    const payload = payloadOf(report);
    const status = statusOfReport(report);
    const media = mediaOf(report);
    const key = reportKey(report, index);
    const clip = buildClipViewer(media, key);
    const uwbAnimation = buildUwbAnimation(report, key, index);
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
            </div>
            <div class="evidence-panel">
              <h3>UWB Movement Around Accident</h3>
              ${uwbAnimation}
            </div>
          </div>
          <details class="subdetails raw-details" data-raw-key="${escapeHtml(key)}" ${openRaw.has(key) ? 'open' : ''}>
            <summary>Raw Accident Data</summary>
            <pre>${escapeHtml(JSON.stringify(payload || report, null, 2))}</pre>
          </details>
        </div>
      </details>`;
  }).join('');
  initClipViewers(root);
  initUwbAnimations(root, reports);
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
  if (refreshInFlight) return;
  refreshInFlight = true;
  try {
    const res = await fetch('/api/state', {cache: 'no-store'});
    const state = await res.json();
    refreshMetrics(state);
    renderDevices(state);
    renderReports(state);
    document.getElementById('raw').textContent = JSON.stringify({lora: state.lora, devices: state.devices}, null, 2);
  } catch (err) {
    document.getElementById('reports').innerHTML = `<div class="empty">${escapeHtml(err)}</div>`;
  } finally {
    refreshInFlight = false;
  }
}
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
"""


class StateStore:
    """Thread-safe memory store shared by HTTP handlers and the LoRa thread."""

    LORA_PRIORITY_WINDOW_SEC = 15.0
    MAX_REPORT_UWB_HISTORY = 20

    def __init__(self, max_reports=30):
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
        devices = {
            device_id: {
            'device_id': device_id,
            'device_type': payload.get('device_type') or payload.get('type') or 'unknown',
            'level': payload.get('level') or payload.get('warning_level') or 'unknown',
            'time': float(payload.get('time', now)),
            'age_sec': 0.0,
            'transport': transport,
            'payload': payload,
            },
        }

        uwb_detail = (
            payload.get('active_sources', {})
            .get('uwb', {})
            .get('detail', {})
        )
        workers = uwb_detail.get('workers', [])
        if isinstance(workers, list):
            for worker in workers:
                if not isinstance(worker, dict):
                    continue
                worker_id = str(
                    worker.get('id') or worker.get('worker_id') or ''
                ).strip()
                if not worker_id:
                    continue
                worker_age = max(0.0, float(worker.get('age_sec') or 0.0))
                worker_payload = dict(worker)
                worker_payload.update({
                    'type': 'worker_ping',
                    'device_id': worker_id,
                    'worker_id': worker_id,
                    'device_type': 'worker',
                    'level': worker.get('warning_level') or 'unknown',
                    'time': now - worker_age,
                    'relay_device_id': device_id,
                })
                devices[worker_id] = {
                    'device_id': worker_id,
                    'device_type': 'worker',
                    'level': worker_payload['level'],
                    'time': worker_payload['time'],
                    'age_sec': worker_age,
                    'transport': f'{transport}-relay',
                    'payload': worker_payload,
                }

        with self.lock:
            for candidate_id, candidate in devices.items():
                existing = self.devices.get(candidate_id)
                lora_is_fresh = (
                    existing is not None
                    and existing.get('transport') == 'lora'
                    and now - float(existing.get('time', 0.0))
                    <= self.LORA_PRIORITY_WINDOW_SEC
                )
                if lora_is_fresh and candidate.get('transport') != 'lora':
                    # Preserve a fresh direct LoRa worker ping as the primary
                    # view, while still attaching the richer HTTP relay payload.
                    enriched_payload = dict(candidate.get('payload') or {})
                    enriched_payload['primary_transport'] = 'lora'
                    enriched_payload['lora_payload'] = existing.get('payload', {})
                    existing = dict(existing)
                    existing['payload'] = enriched_payload
                    self.devices[candidate_id] = existing
                    continue
                self.devices[candidate_id] = candidate

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

    @classmethod
    def _limited_samples(cls, samples):
        if len(samples) <= cls.MAX_REPORT_UWB_HISTORY:
            return samples
        if cls.MAX_REPORT_UWB_HISTORY <= 1:
            return samples[-cls.MAX_REPORT_UWB_HISTORY:]
        last = len(samples) - 1
        steps = cls.MAX_REPORT_UWB_HISTORY - 1
        indices = sorted({
            int(round(index * last / steps))
            for index in range(cls.MAX_REPORT_UWB_HISTORY)
        })
        return [samples[index] for index in indices]

    @staticmethod
    def _compact_ranges(ranges):
        compact = []
        if not isinstance(ranges, list):
            return compact
        for item in ranges:
            if not isinstance(item, dict):
                continue
            compact_item = {}
            for key in ('anchor_id', 'distance_m'):
                if key in item:
                    compact_item[key] = item[key]
            if compact_item:
                compact.append(compact_item)
        return compact

    @classmethod
    def _compact_worker(cls, worker):
        if not isinstance(worker, dict):
            return {}
        keep = (
            'id',
            'worker_id',
            'x',
            'y',
            'display_x',
            'display_y',
            'display_position_valid',
            'distance_to_machine_m',
            'display_distance_to_machine_m',
            'range_count',
            'position_quality',
            'warning_level',
            'display_mode',
        )
        compact = {
            key: worker[key]
            for key in keep
            if key in worker
        }
        for key in ('velocity', 'display_velocity'):
            value = worker.get(key)
            if isinstance(value, dict):
                compact[key] = {
                    subkey: value[subkey]
                    for subkey in ('vx', 'vy', 'speed_mps')
                    if subkey in value
                }
        ranges = cls._compact_ranges(worker.get('ranges'))
        if ranges:
            compact['ranges'] = ranges
        return compact

    @classmethod
    def _compact_uwb_sample(cls, sample):
        if not isinstance(sample, dict):
            return {}
        compact = {}
        for key in ('time', 'frame', 'axis', 'zones'):
            if key in sample:
                compact[key] = sample[key]
        anchors = sample.get('anchors')
        if isinstance(anchors, list):
            compact['anchors'] = [
                {
                    key: anchor[key]
                    for key in ('id', 'x', 'y')
                    if isinstance(anchor, dict) and key in anchor
                }
                for anchor in anchors
                if isinstance(anchor, dict)
            ]
        workers = sample.get('workers')
        if isinstance(workers, list):
            compact['workers'] = [
                cls._compact_worker(worker)
                for worker in workers
                if isinstance(worker, dict)
            ]
        return compact

    @classmethod
    def _compact_evidence(cls, evidence):
        if not isinstance(evidence, dict):
            return evidence
        compact = {}
        if 'window' in evidence:
            compact['window'] = evidence['window']
        snapshot = cls._compact_uwb_sample(evidence.get('uwb_snapshot'))
        if snapshot:
            compact['uwb_snapshot'] = snapshot
        history = evidence.get('uwb_history')
        if isinstance(history, list):
            compact_history = [
                cls._compact_uwb_sample(sample)
                for sample in history
                if isinstance(sample, dict)
            ]
            compact['uwb_history'] = cls._limited_samples(compact_history)
        return compact

    @staticmethod
    def _compact_status(status):
        if not isinstance(status, dict):
            return status
        compact = {
            key: status[key]
            for key in (
                'type',
                'time',
                'device_id',
                'device_type',
                'level',
                'message',
            )
            if key in status
        }
        active_sources = status.get('active_sources')
        if isinstance(active_sources, dict):
            compact['active_sources'] = {}
            for name, source in active_sources.items():
                if not isinstance(source, dict):
                    continue
                compact['active_sources'][name] = {
                    key: source[key]
                    for key in ('level', 'time', 'age_sec')
                    if key in source
                }
        return compact

    @classmethod
    def _compact_report(cls, report):
        # Accident reports can include video and UWB history references. Trim
        # repeated status fields before exposing them through `/api/state`.
        compact = dict(report)
        payload = compact.get('payload')
        if isinstance(payload, dict):
            payload = dict(payload)
            if isinstance(payload.get('status'), dict):
                payload['status'] = cls._compact_status(payload['status'])
            if isinstance(payload.get('evidence'), dict):
                payload['evidence'] = cls._compact_evidence(payload['evidence'])
            compact['payload'] = payload
        return compact

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
                'reports': [
                    self._compact_report(report)
                    for report in self.reports[-self.max_reports:]
                ],
                'lora': dict(self.lora),
            }


class ApiHandler(BaseHTTPRequestHandler):
    """Serve the browser dashboard and accept machine/worker JSON posts."""

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
    allow_reuse_port = True


class Rfm95Receiver(threading.Thread):
    """Poll the local RFM95 and translate received JSON packets into state."""

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

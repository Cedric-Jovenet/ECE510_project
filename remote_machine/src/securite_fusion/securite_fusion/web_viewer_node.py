#!/usr/bin/env python3
import json
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Safety Fusion Viewer</title>
  <style>
    :root { color-scheme: dark; }
    body {
      margin: 0;
      background: #101214;
      color: #edf0f2;
      font-family: Arial, Helvetica, sans-serif;
    }
    main {
      display: grid;
      grid-template-columns: minmax(320px, 1.4fr) minmax(320px, 0.9fr);
      gap: 14px;
      padding: 14px;
      box-sizing: border-box;
      min-height: 100vh;
    }
    section { min-width: 0; }
    h2 {
      margin: 0 0 8px;
      font-size: 16px;
      font-weight: 700;
      letter-spacing: 0;
    }
    img {
      width: 100%;
      max-height: calc(100vh - 95px);
      object-fit: contain;
      background: #050607;
      border: 1px solid #30363d;
      box-sizing: border-box;
    }
    canvas {
      width: 100%;
      aspect-ratio: 1 / 1;
      display: block;
      background: #080a0c;
      border: 1px solid #30363d;
      box-sizing: border-box;
    }
    pre {
      white-space: pre-wrap;
      background: #171a1f;
      border: 1px solid #30363d;
      padding: 10px;
      margin: 10px 0 0;
      max-height: 220px;
      overflow: auto;
      font-size: 12px;
      line-height: 1.35;
    }
    .status-row {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
      margin-top: 10px;
    }
    .metric {
      background: #171a1f;
      border: 1px solid #30363d;
      padding: 8px;
      min-height: 42px;
      box-sizing: border-box;
    }
    .metric b { display: block; font-size: 12px; color: #9aa4af; }
    .metric span { display: block; margin-top: 4px; font-size: 15px; }
    @media (max-width: 900px) {
      main { grid-template-columns: 1fr; }
      img { max-height: 55vh; }
    }
  </style>
</head>
<body>
<main>
  <section>
    <h2>Camera fusion</h2>
    <img id="camera" src="/stream.mjpg" alt="fusion overlay">
    <pre id="status">waiting...</pre>
  </section>
  <section>
    <h2>UWB Map</h2>
    <canvas id="map" width="720" height="720"></canvas>
    <div class="status-row">
      <div class="metric"><b>Workers</b><span id="worker-count">0</span></div>
      <div class="metric"><b>Nearest</b><span id="nearest-worker">n/a</span></div>
      <div class="metric"><b>UWB age</b><span id="uwb-age">n/a</span></div>
    </div>
    <pre id="uwb">waiting...</pre>
  </section>
</main>
<script>
const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
const cameraEl = document.getElementById('camera');
const statusEl = document.getElementById('status');
const uwbEl = document.getElementById('uwb');
const workerCountEl = document.getElementById('worker-count');
const nearestWorkerEl = document.getElementById('nearest-worker');
const uwbAgeEl = document.getElementById('uwb-age');
let statusInFlight = false;
let uwbInFlight = false;

async function refreshStatus() {
  if (statusInFlight) return;
  statusInFlight = true;
  try {
    const res = await fetch('/status.json', { cache: 'no-store' });
    statusEl.textContent = JSON.stringify(await res.json(), null, 2);
  } catch (err) {
    statusEl.textContent = String(err);
  } finally {
    statusInFlight = false;
  }
}

cameraEl.addEventListener('error', () => {
  setTimeout(() => {
    cameraEl.src = `/stream.mjpg?t=${Date.now()}`;
  }, 1000);
});

function worldToCanvas(x, y, scale, cx, cy) {
  return [cx + x * scale, cy - y * scale];
}

function drawCircle(x, y, radius, stroke, fill, lineWidth = 2) {
  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  if (fill) {
    ctx.fillStyle = fill;
    ctx.fill();
  }
  ctx.strokeStyle = stroke;
  ctx.lineWidth = lineWidth;
  ctx.stroke();
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

function drawMap(data) {
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = '#080a0c';
  ctx.fillRect(0, 0, width, height);

  const anchors = data.anchors || [];
  const workers = data.workers || [];
  let maxExtent = 2.0;
  for (const a of anchors) {
    maxExtent = Math.max(maxExtent, Math.abs(a.x), Math.abs(a.y));
  }
  for (const w of workers) {
    const position = workerDisplayPosition(w);
    if (position) {
      maxExtent = Math.max(maxExtent, Math.abs(position[0]), Math.abs(position[1]));
    }
    for (const r of w.ranges || []) {
      const a = anchors.find(item => item.id === r.anchor_id);
      if (!a) continue;
      maxExtent = Math.max(
        maxExtent,
        Math.abs(a.x) + r.distance_m,
        Math.abs(a.y) + r.distance_m
      );
    }
  }
  maxExtent = Math.max(maxExtent, data.zones?.warning_radius_m || 1.5);
  const scale = (Math.min(width, height) * 0.42) / maxExtent;
  const cx = width * 0.5;
  const cy = height * 0.56;

  ctx.strokeStyle = '#1f2933';
  ctx.lineWidth = 1;
  for (let g = -Math.ceil(maxExtent); g <= Math.ceil(maxExtent); g++) {
    const [x0] = worldToCanvas(g, 0, scale, cx, cy);
    ctx.beginPath(); ctx.moveTo(x0, 0); ctx.lineTo(x0, height); ctx.stroke();
    const [, y0] = worldToCanvas(0, g, scale, cx, cy);
    ctx.beginPath(); ctx.moveTo(0, y0); ctx.lineTo(width, y0); ctx.stroke();
  }

  const warning = data.zones?.warning_radius_m || 1.5;
  const critical = data.zones?.critical_radius_m || 0.5;
  drawCircle(cx, cy, warning * scale, '#9d7a23', 'rgba(255,184,28,0.08)', 2);
  drawCircle(cx, cy, critical * scale, '#8d2b2b', 'rgba(255,60,60,0.10)', 2);

  ctx.fillStyle = '#d8dee9';
  ctx.fillRect(cx - 16, cy - 22, 32, 44);
  ctx.strokeStyle = '#ffffff';
  ctx.strokeRect(cx - 16, cy - 22, 32, 44);
  ctx.fillStyle = '#ffffff';
  ctx.font = '13px Arial';
  ctx.fillText('machine', cx + 22, cy + 5);

  if (!workers.length) {
    ctx.fillStyle = '#9aa4af';
    ctx.font = '16px Arial';
    ctx.fillText('No UWB worker received yet', 24, 34);
    ctx.font = '13px Arial';
    ctx.fillText('Les anchors restent visibles; les cercles apparaitront des qu une distance arrive.', 24, 56);
  }

  for (const a of anchors) {
    const [x, y] = worldToCanvas(a.x, a.y, scale, cx, cy);
    ctx.fillStyle = '#5eead4';
    ctx.beginPath();
    ctx.moveTo(x, y - 9);
    ctx.lineTo(x + 9, y + 8);
    ctx.lineTo(x - 9, y + 8);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = '#d6fff8';
    ctx.fillText(`A${a.id}`, x + 10, y - 8);
  }

  let nearest = null;
  for (const w of workers) {
    const position = workerDisplayPosition(w);
    if (!position) continue;
    const [workerX, workerY] = position;
    const [x, y] = worldToCanvas(workerX, workerY, scale, cx, cy);
    const distance = workerDistance(w);
    const level = w.warning_level || 'clear';
    const color = level === 'critical' ? '#ff3b3b' :
      level === 'warning' ? '#ffb81c' : '#4ade80';
    const partial = w.position_quality && w.position_quality !== 'trilaterated';

    for (const r of w.ranges || []) {
      const a = anchors.find(item => item.id === r.anchor_id);
      if (!a) continue;
      const [ax, ay] = worldToCanvas(a.x, a.y, scale, cx, cy);
      ctx.setLineDash([7, 7]);
      drawCircle(
        ax,
        ay,
        r.distance_m * scale,
        'rgba(125,190,255,0.34)',
        null,
        partial ? 2 : 1
      );
      ctx.setLineDash([]);
      ctx.strokeStyle = 'rgba(180,190,200,0.28)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(x, y); ctx.stroke();
    }

    drawCircle(x, y, partial ? 10 : 12, color, partial ? 'rgba(0,0,0,0)' : color, 2);
    const displayVelocity = w.display_velocity || w.velocity || {};
    const vx = displayVelocity.vx || 0;
    const vy = displayVelocity.vy || 0;
    const speed = displayVelocity.speed_mps || 0;
    if (speed > 0.03) {
      const [ex, ey] = worldToCanvas(
        workerX + vx * 1.2,
        workerY + vy * 1.2,
        scale,
        cx,
        cy
      );
      ctx.strokeStyle = '#ffffff';
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(ex, ey); ctx.stroke();
    }

    ctx.fillStyle = '#ffffff';
    ctx.font = '14px Arial';
    const rangeText = `${w.range_count || (w.ranges || []).length}/${anchors.length}`;
    const modeText = w.display_mode && w.display_mode !== 'uwb' ? ` ${w.display_mode}` : '';
    const qualityText = partial ? ` ${w.position_quality}${modeText}` : modeText;
    const distanceText = distance !== null ? `${distance.toFixed(2)}m` : 'n/a';
    ctx.fillText(
      `${w.id} ${distanceText} ${rangeText}${qualityText}`,
      x + 16,
      y - 10
    );
    if (distance !== null && (!nearest || distance < nearest.distance)) {
      nearest = { id: w.id, distance };
    }
  }

  workerCountEl.textContent = workers.length.toString();
  nearestWorkerEl.textContent = nearest ?
    `${nearest.id} ${nearest.distance.toFixed(2)} m` : 'n/a';
  uwbAgeEl.textContent = data.time ?
    `${Math.max(0, Date.now() / 1000 - data.time).toFixed(1)} s` : 'n/a';
}

async function refreshUwb() {
  if (uwbInFlight) return;
  uwbInFlight = true;
  try {
    const res = await fetch('/uwb.json', { cache: 'no-store' });
    const data = await res.json();
    uwbEl.textContent = JSON.stringify(data, null, 2);
    drawMap(data);
  } catch (err) {
    uwbEl.textContent = String(err);
    drawMap({ anchors: [], workers: [] });
  } finally {
    uwbInFlight = false;
  }
}

setInterval(refreshStatus, 500);
setInterval(refreshUwb, 250);
refreshStatus();
refreshUwb();
</script>
</body>
</html>
"""


class FusionWebViewerNode(Node):
    """Serve the latest ROS image topic as MJPEG for a Windows browser."""

    def __init__(self):
        super().__init__('fusion_web_viewer_node')

        self.declare_parameter('image_topic', '/fusion_overlay')
        self.declare_parameter('status_topic', '/fusion_status')
        self.declare_parameter('uwb_topic', '/uwb/workers')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('clip_buffer_sec', 30.0)
        self.declare_parameter('clip_fps', 4.0)
        self.declare_parameter('clip_archive_dir', '/tmp/securite_fusion_clips')

        self.image_topic = self.get_parameter('image_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.uwb_topic = self.get_parameter('uwb_topic').value
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.clip_buffer_sec = float(self.get_parameter('clip_buffer_sec').value)
        self.clip_fps = max(1.0, float(self.get_parameter('clip_fps').value))
        self.clip_archive_dir = str(self.get_parameter('clip_archive_dir').value)
        self.placeholder_jpeg = self._make_placeholder_jpeg()
        os.makedirs(self.clip_archive_dir, exist_ok=True)

        self.lock = threading.Lock()
        self.latest_jpeg = None
        self.frame_buffer = deque()
        self.recorded_clips = {}
        self.last_buffer_time = 0.0
        self.latest_status = {'message': 'waiting for /fusion_status'}
        self.latest_uwb = {
            'message': 'waiting for /uwb/workers',
            'anchors': [],
            'workers': [],
        }

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(String, self.status_topic, self.status_callback, 10)
        self.create_subscription(String, self.uwb_topic, self.uwb_callback, 10)

        self.server = self._make_server()
        self.server_thread = threading.Thread(
            target=self.server.serve_forever,
            daemon=True,
        )
        self.server_thread.start()
        self.get_logger().info(
            f'Open http://<pi-ip>:{self.port} to view {self.image_topic}'
        )

    def _make_server(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                return

            def do_GET(self):
                path = urlsplit(self.path).path
                if path in ('/', '/index.html'):
                    self._send_html()
                elif path == '/status.json':
                    self._send_status()
                elif path == '/uwb.json':
                    self._send_uwb()
                elif path == '/snapshot.jpg':
                    self._send_snapshot()
                elif path == '/stream.mjpg':
                    self._send_stream()
                elif path == '/clip.mjpg':
                    self._send_clip()
                elif path == '/archive_clip':
                    self._send_archive_clip()
                elif path == '/recorded_clip.mjpg':
                    self._send_recorded_clip()
                elif path == '/recorded_frame.jpg':
                    self._send_recorded_frame()
                else:
                    self.send_error(404)

            def do_HEAD(self):
                path = urlsplit(self.path).path
                if path in ('/', '/index.html'):
                    data = HTML_PAGE.encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', str(len(data)))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                elif path in ('/status.json', '/uwb.json'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                elif path == '/snapshot.jpg':
                    jpeg = node.get_latest_jpeg()
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', str(len(jpeg)))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                else:
                    self.send_error(404)

            def _send_html(self):
                data = HTML_PAGE.encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def _send_status(self):
                with node.lock:
                    data = json.dumps(node.latest_status).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def _send_uwb(self):
                with node.lock:
                    data = json.dumps(node.latest_uwb).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def _send_snapshot(self):
                jpeg = node.get_latest_jpeg()
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(jpeg)

            def _send_stream(self):
                self.send_response(200)
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=frame',
                )
                self.end_headers()
                while True:
                    jpeg = node.get_latest_jpeg()
                    if jpeg is None:
                        time.sleep(0.1)
                        continue
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(
                            f'Content-Length: {len(jpeg)}\r\n\r\n'.encode()
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    time.sleep(0.1)

            def _send_clip(self):
                parsed = urlsplit(self.path)
                params = parse_qs(parsed.query)
                now = time.time()
                try:
                    center = float(params.get('center', [now])[0])
                except (TypeError, ValueError):
                    center = now
                try:
                    before = float(params.get('before', [5.0])[0])
                except (TypeError, ValueError):
                    before = 5.0
                try:
                    after = float(params.get('after', [5.0])[0])
                except (TypeError, ValueError):
                    after = 5.0
                try:
                    fps = float(params.get('fps', [node.clip_fps])[0])
                except (TypeError, ValueError):
                    fps = node.clip_fps

                end_time = center + max(0.0, after)
                while time.time() < end_time:
                    time.sleep(min(0.2, end_time - time.time()))

                frames = node.get_clip_frames(center, before, after)
                if not frames:
                    frames = [(now, node.get_latest_jpeg())]

                self.send_response(200)
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=frame',
                )
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()

                delay = 1.0 / max(1.0, min(12.0, fps))
                for _, jpeg in frames:
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(
                            f'Content-Length: {len(jpeg)}\r\n\r\n'.encode()
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    time.sleep(delay)

            def _send_archive_clip(self):
                parsed = urlsplit(self.path)
                params = parse_qs(parsed.query)
                clip_id = params.get('clip_id', [''])[0]
                now = time.time()
                try:
                    center = float(params.get('center', [now])[0])
                except (TypeError, ValueError):
                    center = now
                try:
                    before = float(params.get('before', [5.0])[0])
                except (TypeError, ValueError):
                    before = 5.0
                try:
                    after = float(params.get('after', [5.0])[0])
                except (TypeError, ValueError):
                    after = 5.0
                try:
                    fps = float(params.get('fps', [node.clip_fps])[0])
                except (TypeError, ValueError):
                    fps = node.clip_fps

                result = node.archive_clip(clip_id, center, before, after, fps)
                data = json.dumps(result, separators=(',', ':')).encode('utf-8')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def _send_recorded_clip(self):
                parsed = urlsplit(self.path)
                params = parse_qs(parsed.query)
                clip_id = params.get('clip_id', [''])[0]
                clip = node.get_recorded_clip(clip_id)
                if not clip:
                    self.send_error(404, 'Recorded clip not found')
                    return

                self.send_response(200)
                self.send_header(
                    'Content-Type',
                    'multipart/x-mixed-replace; boundary=frame',
                )
                self.send_header('Cache-Control', 'private, max-age=3600')
                self.end_headers()

                fps = max(1.0, min(12.0, float(clip.get('fps') or node.clip_fps)))
                delay = 1.0 / fps
                for frame in clip.get('frames', []):
                    jpeg = frame.get('jpeg')
                    if not jpeg:
                        continue
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(
                            f'Content-Length: {len(jpeg)}\r\n\r\n'.encode()
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError):
                        break
                    time.sleep(delay)

            def _send_recorded_frame(self):
                parsed = urlsplit(self.path)
                params = parse_qs(parsed.query)
                clip_id = params.get('clip_id', [''])[0]
                try:
                    index = int(params.get('index', [0])[0])
                except (TypeError, ValueError):
                    index = 0
                clip = node.get_recorded_clip(clip_id)
                frames = clip.get('frames', []) if clip else []
                if index < 0 or index >= len(frames):
                    self.send_error(404, 'Recorded frame not found')
                    return
                jpeg = frames[index].get('jpeg')
                if not jpeg:
                    self.send_error(404, 'Recorded frame not found')
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', str(len(jpeg)))
                self.send_header('Cache-Control', 'private, max-age=3600')
                self.end_headers()
                self.wfile.write(jpeg)

        return ThreadingHTTPServer((self.host, self.port), Handler)

    def image_callback(self, msg):
        try:
            rgb = self._image_to_rgb(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc), throttle_duration_sec=2.0)
            return

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(
            '.jpg',
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return
        now = time.time()
        frame = encoded.tobytes()
        with self.lock:
            self.latest_jpeg = frame
            if now - self.last_buffer_time >= 1.0 / self.clip_fps:
                self.frame_buffer.append((now, frame))
                self.last_buffer_time = now
                cutoff = now - self.clip_buffer_sec
                while self.frame_buffer and self.frame_buffer[0][0] < cutoff:
                    self.frame_buffer.popleft()

    def status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            status = {'raw': msg.data}
        with self.lock:
            self.latest_status = status

    def uwb_callback(self, msg):
        try:
            uwb = json.loads(msg.data)
        except json.JSONDecodeError:
            uwb = {'raw': msg.data, 'anchors': [], 'workers': []}
        with self.lock:
            self.latest_uwb = uwb

    def get_latest_jpeg(self):
        with self.lock:
            return self.latest_jpeg or self.placeholder_jpeg

    def get_clip_frames(self, center_time, before_sec, after_sec):
        start_time = float(center_time) - max(0.0, float(before_sec))
        end_time = float(center_time) + max(0.0, float(after_sec))
        with self.lock:
            return [
                (timestamp, jpeg)
                for timestamp, jpeg in self.frame_buffer
                if start_time <= timestamp <= end_time
            ]

    def archive_clip(self, clip_id, center_time, before_sec, after_sec, fps):
        safe_id = self._safe_clip_id(clip_id)
        center = float(center_time)
        before = max(0.0, float(before_sec))
        after = max(0.0, float(after_sec))
        end_time = center + after
        while time.time() < end_time:
            time.sleep(min(0.2, end_time - time.time()))

        frames = self.get_clip_frames(center, before, after)
        if not frames:
            return {
                'ok': False,
                'clip_id': safe_id,
                'error': 'no buffered frames for requested accident window',
            }

        clip_dir = os.path.join(self.clip_archive_dir, safe_id)
        os.makedirs(clip_dir, exist_ok=True)
        manifest_frames = []
        memory_frames = []
        for index, (timestamp, jpeg) in enumerate(frames):
            filename = f'frame_{index:06d}.jpg'
            path = os.path.join(clip_dir, filename)
            with open(path, 'wb') as handle:
                handle.write(jpeg)
            manifest_frames.append({'time': timestamp, 'file': filename})
            memory_frames.append({'time': timestamp, 'jpeg': jpeg})

        manifest = {
            'clip_id': safe_id,
            'created_time': time.time(),
            'center_time': center,
            'before_sec': before,
            'after_sec': after,
            'fps': max(1.0, min(12.0, float(fps))),
            'frames': manifest_frames,
        }
        with open(os.path.join(clip_dir, 'manifest.json'), 'w', encoding='utf-8') as handle:
            json.dump(manifest, handle, separators=(',', ':'))
        with self.lock:
            self.recorded_clips[safe_id] = dict(manifest, frames=memory_frames)
        return {
            'ok': True,
            'clip_id': safe_id,
            'frame_count': len(memory_frames),
            'center_time': center,
            'before_sec': before,
            'after_sec': after,
        }

    def get_recorded_clip(self, clip_id):
        safe_id = self._safe_clip_id(clip_id)
        with self.lock:
            clip = self.recorded_clips.get(safe_id)
        if clip:
            return clip

        clip_dir = os.path.join(self.clip_archive_dir, safe_id)
        manifest_path = os.path.join(clip_dir, 'manifest.json')
        if not os.path.isfile(manifest_path):
            return None
        try:
            with open(manifest_path, 'r', encoding='utf-8') as handle:
                manifest = json.load(handle)
            frames = []
            for frame in manifest.get('frames', []):
                path = os.path.join(clip_dir, frame.get('file', ''))
                with open(path, 'rb') as handle:
                    frames.append({
                        'time': frame.get('time'),
                        'jpeg': handle.read(),
                    })
            clip = dict(manifest, frames=frames)
        except (OSError, json.JSONDecodeError):
            return None
        with self.lock:
            self.recorded_clips[safe_id] = clip
        return clip

    @staticmethod
    def _safe_clip_id(clip_id):
        text = str(clip_id or '').strip()
        safe = ''.join(
            char if char.isalnum() or char in '._-' else '_'
            for char in text
        )
        return (safe[:96] or f'clip-{int(time.time() * 1000)}')

    def _make_placeholder_jpeg(self):
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        image[:] = (12, 15, 18)
        cv2.putText(
            image,
            'Camera inactive',
            (42, 150),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (230, 235, 240),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            'La carte UWB reste disponible a droite.',
            (42, 195),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (150, 160, 170),
            1,
            cv2.LINE_AA,
        )
        ok, encoded = cv2.imencode(
            '.jpg',
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return b''
        return encoded.tobytes()

    def _image_to_rgb(self, msg):
        if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
            raise ValueError(f'Unsupported image encoding: {msg.encoding}')
        if msg.encoding == 'mono8':
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height,
                msg.width,
            )
            return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height,
            msg.width,
            3,
        )
        if msg.encoding == 'bgr8':
            return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image.copy()

    def destroy_node(self):
        if getattr(self, 'server', None) is not None:
            self.server.shutdown()
            self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FusionWebViewerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

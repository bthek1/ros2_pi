// The page's whole client. No framework, no bundler, no CDN.
//
// Three things happen here and nothing else: a WebSocket is read and dispatched
// on its leading channel byte, the panel is redrawn from whatever the last
// `stats` message said, and a triangle soup is drawn with raw WebGL.
//
// **This file computes nothing about the pipeline.** Every figure it shows is a
// figure some node measured about itself and published on `/pipeline/stats`, so
// the panel and `ros2 topic echo` cannot disagree. Where it would be easy to
// derive a number here — a drop *percentage*, say — it is formatted, not
// calculated from two unrelated counters.
'use strict';

// Must match `enum class Channel` in web_server.hpp. One byte at the front of
// every payload, JSON included, so the client has one code path instead of
// branching on frame type *and* on a field inside the JSON — two things that can
// disagree.
const CH = { STATS: 1, RGB: 2, DEPTH: 3, POSE: 4, MESH: 5 };

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------------------
// The socket
// ---------------------------------------------------------------------------

let socket = null;
let retry = 500;

function connect() {
  const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
  socket = new WebSocket(url);
  socket.binaryType = 'arraybuffer';

  socket.onopen = () => {
    retry = 500;
    $('link').textContent = 'live';
    $('link').className = 'pill up';
  };
  socket.onclose = () => {
    $('link').textContent = 'disconnected';
    $('link').className = 'pill down';
    // Backoff, capped. The node is allowed to die — that is the promise it
    // makes — and a page that reconnects every 20 ms while it is down is a page
    // that makes restarting it slower.
    setTimeout(connect, retry);
    retry = Math.min(retry * 2, 5000);
  };
  socket.onmessage = (event) => {
    const bytes = new Uint8Array(event.data);
    if (bytes.length < 1) { return; }
    const body = bytes.subarray(1);
    switch (bytes[0]) {
      case CH.STATS: onStats(JSON.parse(new TextDecoder().decode(body))); break;
      case CH.RGB: showImage($('rgb'), $('rgb-stale'), body); break;
      case CH.DEPTH: showImage($('depth'), $('depth-stale'), body); break;
      case CH.POSE: onPose(JSON.parse(new TextDecoder().decode(body))); break;
      case CH.MESH: onMesh(body); break;
      default: break;   // An unknown channel is a newer server, not an error.
    }
  };
}

// ---------------------------------------------------------------------------
// Images
// ---------------------------------------------------------------------------

const lastSeen = { rgb: 0, depth: 0 };

function showImage(img, staleFlag, bytes) {
  // A Blob URL straight from the bytes: no base64, which would cost 33% for
  // nothing. The previous URL is revoked or the page leaks one object per frame
  // — at 10 Hz that is 36 000 an hour, and the symptom is a tab that gets slower
  // the longer it is left open, which is a miserable thing to debug.
  const url = URL.createObjectURL(new Blob([bytes], { type: 'image/jpeg' }));
  const old = img.dataset.url;
  img.src = url;
  img.dataset.url = url;
  if (old) { URL.revokeObjectURL(old); }
  lastSeen[img.id] = performance.now();
  staleFlag.classList.remove('on');
}

// Staleness on **receipt**, here as on the server: `performance.now()` is this
// browser's own monotonic clock and owes nothing to any other machine's.
setInterval(() => {
  const now = performance.now();
  $('rgb-stale').classList.toggle('on', now - lastSeen.rgb > 2000);
  $('depth-stale').classList.toggle('on', now - lastSeen.depth > 2000);
}, 500);

// ---------------------------------------------------------------------------
// The stage table
// ---------------------------------------------------------------------------

function fmt(value, digits) {
  return (value === null || value === undefined || !isFinite(value)) ? '—' : value.toFixed(digits);
}

function onStats(stats) {
  $('uptime').textContent = hms(stats.uptime_s);
  $('clients').textContent = stats.clients + (stats.clients === 1 ? ' viewer' : ' viewers');
  $('ws-dropped').textContent = stats.ws_dropped + ' frames dropped to slow clients';

  const rows = stats.stages.map((s) => {
    const lost = s.dropped_in_transport;
    return `<tr class="${s.stale ? 'stale' : ''}">
      <td>${esc(s.stage)}${s.stale ? ' <span class="stale on">STALE</span>' : ''}</td>
      <td>${fmt(s.rate_hz, 1)}</td>
      <td>${fmt(s.latency_ms, 1)}</td>
      <td class="by-design">${s.dropped_by_design}</td>
      <td class="lost ${lost > 0 ? 'some' : ''}">${lost}</td>
      <td class="detail">${esc(s.detail)}</td>
    </tr>`;
  });
  $('stage-rows').innerHTML = rows.length ?
    rows.join('') : '<tr class="empty"><td colspan="6">nothing on /pipeline/stats yet</td></tr>';

  const by = Object.fromEntries(stats.stages.map((s) => [s.stage, s]));
  $('hz-depth').textContent = by.depth ? fmt(by.depth.rate_hz, 1) + ' Hz' : '—';
  $('hz-capture').textContent = by.capture ? fmt(by.capture.rate_hz, 1) + ' Hz' : '—';
}

function hms(seconds) {
  const s = Math.floor(seconds || 0);
  const m = Math.floor(s / 60);
  return m >= 60 ? `${Math.floor(m / 60)}h${String(m % 60).padStart(2, '0')}m`
    : `${m}m${String(s % 60).padStart(2, '0')}s`;
}

// `detail` is free-form text written by another node and it goes into the
// document. It is trusted — it comes off a topic on this machine's own ROS graph
// — but escaping it costs nothing and an unescaped `<` would silently eat the
// rest of the row.
function esc(text) {
  return String(text === undefined ? '' : text).replace(/[&<>"]/g,
    (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

// ---------------------------------------------------------------------------
// The buttons
// ---------------------------------------------------------------------------

function action(name, note) {
  $('action-note').textContent = note;
  fetch('/action/' + name, { method: 'POST' })
    .then((r) => r.text())
    .then((text) => { $('action-note').textContent = text.trim(); })
    .catch((e) => { $('action-note').textContent = 'failed: ' + e; });
}

$('save').onclick = () => action('save_mesh', 'saving…');
$('reset').onclick = () => {
  // Confirmed, because it throws away a session's work and there is no undo —
  // the volume is process memory and nothing else holds a copy of it.
  if (confirm('Throw away the map and start again? This cannot be undone.')) {
    action('reset_map', 'resetting…');
  }
};

// ---------------------------------------------------------------------------
// The 3D view, in raw WebGL
// ---------------------------------------------------------------------------

const canvas = $('gl');
const gl = canvas.getContext('webgl', { antialias: true, alpha: false });

const VERT = `
attribute vec3 position;
attribute vec3 colour;
uniform mat4 mvp;
varying vec3 vColour;
varying float vDepth;
void main() {
  gl_Position = mvp * vec4(position, 1.0);
  vColour = colour;
  vDepth = gl_Position.z / gl_Position.w;
  gl_PointSize = 3.0;
}`;

const FRAG = `
precision mediump float;
varying vec3 vColour;
varying float vDepth;
void main() {
  // A cheap depth cue instead of a lighting model: the mesh has vertex colours
  // from the camera and no normals, so shading it would mean computing normals
  // here for a surface that is already coloured by what it looked like.
  float fade = clamp(1.0 - vDepth * 0.35, 0.45, 1.0);
  gl_FragColor = vec4(vColour * fade, 1.0);
}`;

let program = null;
let meshBuffers = null;
let meshCount = 0;
let trailBuffer = null;
let trailCount = 0;

const camera = { yaw: 2.4, pitch: 0.5, distance: 6.0, target: [0, 0, 0] };

function compile(type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    throw new Error(gl.getShaderInfoLog(shader));
  }
  return shader;
}

function initGl() {
  if (!gl) {
    $('scene-note').textContent = 'no WebGL in this browser — the panel still works';
    return false;
  }
  program = gl.createProgram();
  gl.attachShader(program, compile(gl.VERTEX_SHADER, VERT));
  gl.attachShader(program, compile(gl.FRAGMENT_SHADER, FRAG));
  gl.linkProgram(program);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    throw new Error(gl.getProgramInfoLog(program));
  }
  gl.enable(gl.DEPTH_TEST);
  gl.clearColor(0.055, 0.063, 0.078, 1.0);
  meshBuffers = { position: gl.createBuffer(), colour: gl.createBuffer() };
  trailBuffer = { position: gl.createBuffer(), colour: gl.createBuffer() };
  return true;
}

function onMesh(bytes) {
  // The payload is [u32 vertex count][f32 xyz ...][u8 rgb ...]. A TRIANGLE_LIST
  // Marker is already three vertices per triangle with no index reuse, so the
  // indices would be 0,1,2,… and sending them would be a third of the payload
  // saying nothing.
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const count = view.getUint32(0, true);
  if (count === 0) { return; }

  // `slice`, not `subarray`: a Float32Array view needs 4-byte alignment and this
  // one starts at offset 4 of a buffer whose own offset is 1 (the channel byte).
  // A subarray would throw on some inputs and not others, which is the worst
  // kind of intermittent.
  const positions = new Float32Array(bytes.buffer.slice(
    bytes.byteOffset + 4, bytes.byteOffset + 4 + count * 12));
  const colours = new Uint8Array(bytes.buffer.slice(
    bytes.byteOffset + 4 + count * 12, bytes.byteOffset + 4 + count * 12 + count * 3));

  gl.bindBuffer(gl.ARRAY_BUFFER, meshBuffers.position);
  gl.bufferData(gl.ARRAY_BUFFER, positions, gl.DYNAMIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, meshBuffers.colour);
  gl.bufferData(gl.ARRAY_BUFFER, colours, gl.DYNAMIC_DRAW);
  meshCount = count;

  $('tris').textContent = (count / 3).toLocaleString();
  $('scene-note').textContent = '';

  // Frame the surface the first time one arrives, so the default camera is not
  // pointing at empty space in a volume whose scale is arbitrary.
  if (!camera.framed) {
    let cx = 0, cy = 0, cz = 0;
    for (let i = 0; i < count; ++i) {
      cx += positions[i * 3]; cy += positions[i * 3 + 1]; cz += positions[i * 3 + 2];
    }
    camera.target = [cx / count, cy / count, cz / count];
    let spread = 0;
    for (let i = 0; i < count; i += 97) {
      spread = Math.max(spread, Math.abs(positions[i * 3] - camera.target[0]));
    }
    camera.distance = Math.max(2.0, spread * 2.5);
    camera.framed = true;
  }
}

function onPose(pose) {
  if (!pose.have || !gl) { return; }
  const trail = pose.trail || [];
  trailCount = trail.length / 3;
  if (trailCount < 2) { return; }
  const positions = new Float32Array(trail);
  const colours = new Uint8Array(trailCount * 3);
  for (let i = 0; i < trailCount; ++i) {
    // Oldest dim, newest bright: the direction of travel is readable without a
    // legend, which is the one thing a trail has to say.
    const t = i / Math.max(1, trailCount - 1);
    colours[i * 3] = 60 + 195 * t;
    colours[i * 3 + 1] = 120 + 46 * t;
    colours[i * 3 + 2] = 255;
  }
  gl.bindBuffer(gl.ARRAY_BUFFER, trailBuffer.position);
  gl.bufferData(gl.ARRAY_BUFFER, positions, gl.DYNAMIC_DRAW);
  gl.bindBuffer(gl.ARRAY_BUFFER, trailBuffer.colour);
  gl.bufferData(gl.ARRAY_BUFFER, colours, gl.DYNAMIC_DRAW);
}

// --- The smallest matrix library that does this job ------------------------

function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2);
  return [f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) / (near - far), -1,
    0, 0, 2 * far * near / (near - far), 0];
}

function lookAt(eye, at, up) {
  const z = normalise([eye[0] - at[0], eye[1] - at[1], eye[2] - at[2]]);
  const x = normalise(cross(up, z));
  const y = cross(z, x);
  return [x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0,
    -dot(x, eye), -dot(y, eye), -dot(z, eye), 1];
}

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
function normalise(v) {
  const n = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / n, v[1] / n, v[2] / n];
}

function multiply(a, b) {
  const out = new Array(16).fill(0);
  for (let i = 0; i < 4; ++i) {
    for (let j = 0; j < 4; ++j) {
      for (let k = 0; k < 4; ++k) { out[j * 4 + i] += a[k * 4 + i] * b[j * 4 + k]; }
    }
  }
  return out;
}

function draw() {
  requestAnimationFrame(draw);
  if (!gl || !program) { return; }

  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const ratio = window.devicePixelRatio || 1;
  if (canvas.width !== width * ratio || canvas.height !== height * ratio) {
    canvas.width = width * ratio;
    canvas.height = height * ratio;
  }
  gl.viewport(0, 0, canvas.width, canvas.height);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  if (meshCount === 0 && trailCount === 0) { return; }

  const eye = [
    camera.target[0] + camera.distance * Math.cos(camera.pitch) * Math.cos(camera.yaw),
    camera.target[1] + camera.distance * Math.cos(camera.pitch) * Math.sin(camera.yaw),
    camera.target[2] + camera.distance * Math.sin(camera.pitch),
  ];
  // Z up, as the map frame is. Using Y up here would make a level room look
  // like a wall, which is a confusing first impression of a correct mesh.
  const mvp = multiply(
    perspective(Math.PI / 4, Math.max(0.001, width / height), 0.05, 200),
    lookAt(eye, camera.target, [0, 0, 1]));

  gl.useProgram(program);
  gl.uniformMatrix4fv(gl.getUniformLocation(program, 'mvp'), false, new Float32Array(mvp));
  const position = gl.getAttribLocation(program, 'position');
  const colour = gl.getAttribLocation(program, 'colour');
  gl.enableVertexAttribArray(position);
  gl.enableVertexAttribArray(colour);

  if (meshCount > 0) {
    gl.bindBuffer(gl.ARRAY_BUFFER, meshBuffers.position);
    gl.vertexAttribPointer(position, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, meshBuffers.colour);
    gl.vertexAttribPointer(colour, 3, gl.UNSIGNED_BYTE, true, 0, 0);
    gl.drawArrays(gl.TRIANGLES, 0, meshCount);
  }
  if (trailCount > 1) {
    gl.bindBuffer(gl.ARRAY_BUFFER, trailBuffer.position);
    gl.vertexAttribPointer(position, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, trailBuffer.colour);
    gl.vertexAttribPointer(colour, 3, gl.UNSIGNED_BYTE, true, 0, 0);
    gl.drawArrays(gl.LINE_STRIP, 0, trailCount);
  }
}

// --- Orbit ------------------------------------------------------------------

let dragging = null;
canvas.addEventListener('mousedown', (e) => {
  dragging = { x: e.clientX, y: e.clientY, pan: e.shiftKey };
});
window.addEventListener('mouseup', () => { dragging = null; });
window.addEventListener('mousemove', (e) => {
  if (!dragging) { return; }
  const dx = e.clientX - dragging.x;
  const dy = e.clientY - dragging.y;
  dragging.x = e.clientX;
  dragging.y = e.clientY;
  if (dragging.pan) {
    const scale = camera.distance * 0.0016;
    camera.target[0] -= dx * scale * Math.sin(camera.yaw);
    camera.target[1] += dx * scale * Math.cos(camera.yaw);
    camera.target[2] += dy * scale;
  } else {
    camera.yaw -= dx * 0.006;
    // Clamped short of straight up: at exactly the pole the up vector and the
    // view direction are parallel and lookAt produces a NaN matrix, which
    // renders as the scene vanishing.
    camera.pitch = Math.max(-1.5, Math.min(1.5, camera.pitch + dy * 0.006));
  }
});
canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  camera.distance = Math.max(0.2, Math.min(200, camera.distance * (1 + Math.sign(e.deltaY) * 0.12)));
}, { passive: false });

if (initGl()) { requestAnimationFrame(draw); }
connect();

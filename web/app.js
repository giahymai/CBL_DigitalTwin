// app.js — render the farm world on a <canvas>.
//
// The MAP constant below is the exact output of
//   python3 scripts/world_to_canvas.py worlds/new_world.world
// embedded inline so this page works from file:// (browsers block fetch()
// on local files). Re-run that script and paste the new JSON into MAP if
// you change the world.

const MAP = {
  "source": "worlds/new_world.world",
  "bounds": {
    "min_x": -1.055,
    "min_y": -4.165565579247321,
    "max_x": 2.6531152505346234,
    "max_y": 1.3066399574279788
  },
  "shapes": [
    { "type": "box", "name": "box",      "x": -1.024209976196289,   "y":  0.6777790188789368, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_1",    "x": -1.03,                "y":  0.07,               "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_2",    "x": -1.02,                "y": -0.52,               "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_3",    "x": -1.0226199626922607,  "y": -1.1138999462127686, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_4",    "x": -1.0205800533294678,  "y": -1.7115800380706787, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_5",    "x": -1.023419976234436,   "y": -2.311429977416992,  "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_6",    "x": -1.0233800411224365,  "y": -2.9101500511169434, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_7",    "x": -1.0221500396728516,  "y": -3.5036098957061768, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8",    "x": -0.6974999904632568,  "y":  1.0066399574279787, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_1",  "x": -0.10089799761772156, "y":  1.0029499530792234, "w": 0.05, "h": 0.6, "yaw": 1.56841005300998,    "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_2",  "x":  0.4999748468399049,  "y":  0.9984285831451414, "w": 0.05, "h": 0.6, "yaw": 1.570000084190661,   "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_3",  "x":  1.1022270375607515,  "y":  0.9991031217496191, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_4",  "x":  1.701962914249203,   "y":  0.9965046031326728, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5",  "x":  2.300076339498841,   "y":  0.9913790535651015, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_9",    "x":  2.626252963941734,   "y":  0.6655293306074757, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_10",   "x":  2.625702667551475,   "y":  0.06519554283408446,"w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_11",   "x":  2.627592792649916,   "y": -0.5350272778976182, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_12",   "x":  2.6281152505346235,  "y": -1.1352186579326506, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_13",   "x":  2.626794870493436,   "y": -1.7371728259073804, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_14",   "x":  2.626761430535237,   "y": -2.33662920422992,   "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_15",   "x":  2.6280006269886167,  "y": -2.938907322034053,  "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_16",   "x":  2.627476467678176,   "y": -3.5421945732274764, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_1","x":  2.3170975934162765,  "y": -3.865565579247321,  "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_2","x":  1.7183380653296056,  "y": -3.862349183830966,  "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_3","x":  1.1190764522698835,  "y": -3.8600206948799998, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_4","x":  0.5201677522342384,  "y": -3.856848256590511,  "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_5","x": -0.08012115888226501, "y": -3.8538894517536315, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_6","x": -0.6798431661426017,  "y": -3.8498007437380806, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_7","x": -0.7179923117618596,  "y": -1.7295482278429848, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_9","x":  2.3106882813119443,  "y": -0.8030153485568308, "w": 0.05, "h": 0.6, "yaw": 1.5658000872875657,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_1_1",  "x":  0.7740235284633101,  "y":  0.6689040995166549, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_1_2_2","x": -0.7463012851378354,  "y":  0.778091460786658,  "w": 0.05, "h": 0.6, "yaw": -0.9211320022812256, "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_1_2_3","x":  2.3426086092275002,  "y": -3.6236037284401896, "w": 0.05, "h": 0.6, "yaw": -0.9211320022812256, "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_5_8_3","x": -0.7263325022774926,"y": -1.9181128806837573, "w": 0.05, "h": 0.6, "yaw": 2.0823900577787207,  "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_1_1_1","x":  0.9710946682377841,  "y": -1.3696296970979935, "w": 0.05, "h": 0.6, "yaw": 0.0,                 "color": "rgba(178,178,178,1)" },
    { "type": "box", "name": "box_8_1_1","x":  0.6914829712550123,  "y": -1.0442157666892553, "w": 0.05, "h": 0.6, "yaw": 1.56841005300998,    "color": "rgba(178,178,178,1)" },
    { "type": "circle", "name": "spray_zone_A",     "x":  0.0, "y":  0.4, "r": 0.35, "color": "rgba(230, 26, 26, 1)" },
    { "type": "circle", "name": "fertilize_zone_B", "x":  1.8, "y":  0.4, "r": 0.35, "color": "rgba( 26,204, 26, 1)" },
    { "type": "circle", "name": "spray_zone_C",     "x":  0.0, "y": -3.0, "r": 0.35, "color": "rgba(230, 26, 26, 1)" },
    { "type": "circle", "name": "fertilize_zone_D", "x":  1.6, "y": -3.0, "r": 0.35, "color": "rgba( 26,204, 26, 1)" }
  ]
};

// World-frame padding around the map bounds, in metres. Keeps the outer
// walls from sitting flush against the canvas edge.
const PADDING_M = 0.5;

// TurtleBot3 Burger footprint — 138 mm wide chassis ≈ 0.069 m radius.
// Drawn as a single filled disc to match what the LiDAR plane sees.
const ROBOT_RADIUS_M  = 0.069;
const ROBOT_FILL      = "rgba(60, 130, 220, 0.92)";
const ROBOT_STROKE    = "rgba(0, 0, 0, 0.65)";
// Heading indicator — a line from disc centre forward, slightly past
// the disc edge so it reads as a clear "this way is +X".
const HEADING_LEN_M   = ROBOT_RADIUS_M * 1.35;
const HEADING_STROKE  = "rgba(20, 20, 20, 0.95)";
// LaserScan dots — small red dots at every returned (r, θ).
const LIDAR_FILL      = "rgba(220, 35, 35, 0.78)";
const LIDAR_DOT_PX    = 1.6;

// Robot's spawn pose. Must match gazebo_nav2_demo.launch.py's x_pose /
// y_pose (and nav2_sim.yaml's initial_pose). Used as the drawing
// fallback until AMCL publishes its first /amcl_pose, so the disc is
// already on screen the moment the page loads.
const INITIAL_POSE = {
  position: { x: 1.5, y: -2.0 },
  yaw: 0.0,
};

// Recomputed each draw — kept in module scope so a future click/zoom
// handler can convert pointer positions back to world coords.
const view = { offsetX: 0, offsetY: 0, scale: 1 };

// Live canvas refs — set in DOMContentLoaded so handleStateChange can
// repaint on every SSE tick without re-querying the DOM.
let canvasEl = null;
let ctxRef   = null;

function fitView(canvas) {
  // metres-per-pixel scale + offset so the whole world (plus padding) fits
  // inside the canvas, centred.
  const b = MAP.bounds;
  const dx = (b.max_x - b.min_x) + 2 * PADDING_M;
  const dy = (b.max_y - b.min_y) + 2 * PADDING_M;
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.width  / dpr;
  const H = canvas.height / dpr;
  const scale = Math.min(W / dx, H / dy);
  const drawnW = dx * scale;
  const drawnH = dy * scale;
  view.scale   = scale;
  view.offsetX = (W - drawnW) / 2 - (b.min_x - PADDING_M) * scale;
  // Flip Y: world Y is up, canvas Y is down.
  view.offsetY = (H + drawnH) / 2 + (b.min_y - PADDING_M) * scale;
}

function worldToCanvas(x, y) {
  return [x * view.scale + view.offsetX,
          -y * view.scale + view.offsetY];
}

function drawGrid(ctx, canvas) {
  const b = MAP.bounds;
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.width  / dpr;
  const H = canvas.height / dpr;
  ctx.save();
  ctx.strokeStyle = "rgba(0,0,0,0.08)";
  ctx.lineWidth = 1;
  const x0 = Math.floor(b.min_x - PADDING_M);
  const x1 = Math.ceil (b.max_x + PADDING_M);
  const y0 = Math.floor(b.min_y - PADDING_M);
  const y1 = Math.ceil (b.max_y + PADDING_M);
  for (let gx = x0; gx <= x1; gx++) {
    const cx = worldToCanvas(gx, 0)[0];
    ctx.beginPath();
    ctx.moveTo(cx, 0);
    ctx.lineTo(cx, H);
    ctx.stroke();
  }
  for (let gy = y0; gy <= y1; gy++) {
    const cy = worldToCanvas(0, gy)[1];
    ctx.beginPath();
    ctx.moveTo(0, cy);
    ctx.lineTo(W, cy);
    ctx.stroke();
  }
  // Axes — solid through world (0, 0).
  ctx.strokeStyle = "rgba(0,0,0,0.25)";
  ctx.lineWidth = 1.5;
  const [ox, oy] = worldToCanvas(0, 0);
  ctx.beginPath(); ctx.moveTo(ox, 0); ctx.lineTo(ox, H); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0, oy); ctx.lineTo(W, oy); ctx.stroke();
  ctx.restore();
}

function drawShape(ctx, s) {
  ctx.fillStyle   = s.color;
  ctx.strokeStyle = "rgba(0,0,0,0.5)";
  ctx.lineWidth   = 1;

  if (s.type === "box") {
    const [cx, cy] = worldToCanvas(s.x, s.y);
    const w = s.w * view.scale;
    const h = s.h * view.scale;
    ctx.save();
    ctx.translate(cx, cy);
    // World yaw is CCW about +Z; canvas Y is flipped, so flip the rotation
    // sign too — otherwise rotated walls would mirror visually.
    ctx.rotate(-(s.yaw || 0));
    ctx.fillRect(-w / 2, -h / 2, w, h);
    ctx.strokeRect(-w / 2, -h / 2, w, h);
    ctx.restore();
    return;
  }

  if (s.type === "circle") {
    const [cx, cy] = worldToCanvas(s.x, s.y);
    const r = s.r * view.scale;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
    return;
  }
}

function drawLidar(ctx) {
  // /scan_filtered arrives in the robot's base_scan frame: each entry is
  // a distance at angle_min + i * angle_increment, measured CCW from the
  // sensor's +X axis. To plot in the map frame we add the robot's yaw
  // and offset by its position. The 4 cm base_link -> base_scan offset
  // on the TB3 Burger is ignored — invisible at this canvas scale.
  const scan = STATE.scan;
  if (!scan || !Array.isArray(scan.ranges) || scan.ranges.length === 0) return;
  const pose = (STATE.amcl_pose && STATE.amcl_pose.pose) || INITIAL_POSE;
  const px   = pose.position.x;
  const py   = pose.position.y;
  const yaw  = (pose.yaw != null) ? pose.yaw : 0.0;
  const rMax = scan.range_max || Infinity;
  const rMin = scan.range_min || 0.0;

  ctx.save();
  ctx.fillStyle = LIDAR_FILL;
  for (let i = 0; i < scan.ranges.length; i++) {
    const r = scan.ranges[i];
    // null = NaN/Inf from the original message; out-of-range rings are
    // not real obstacle returns either.
    if (r == null || r <= rMin || r > rMax) continue;
    const theta = scan.angle_min + i * scan.angle_increment + yaw;
    const wx = px + r * Math.cos(theta);
    const wy = py + r * Math.sin(theta);
    const [cx, cy] = worldToCanvas(wx, wy);
    // fillRect with a half-pixel offset is ~3x faster than arc() and
    // visually indistinguishable at this dot size.
    ctx.fillRect(cx - LIDAR_DOT_PX, cy - LIDAR_DOT_PX,
                 LIDAR_DOT_PX * 2, LIDAR_DOT_PX * 2);
  }
  ctx.restore();
}

function drawRobot(ctx) {
  // AMCL pose is in the map frame, which is what the canvas is in. /odom
  // lives in the odom frame and would render at the wrong spot without
  // applying the map->odom transform, so we deliberately ignore it.
  // Before AMCL converges, fall back to INITIAL_POSE so the disc is
  // visible from the moment the page loads.
  const pose = (STATE.amcl_pose && STATE.amcl_pose.pose) || INITIAL_POSE;
  const px  = pose.position.x;
  const py  = pose.position.y;
  const yaw = (pose.yaw != null) ? pose.yaw : 0.0;

  const [cx, cy] = worldToCanvas(px, py);
  const r = ROBOT_RADIUS_M * view.scale;

  ctx.save();
  // Body disc.
  ctx.fillStyle   = ROBOT_FILL;
  ctx.strokeStyle = ROBOT_STROKE;
  ctx.lineWidth   = 1.5;
  ctx.beginPath();
  ctx.arc(cx, cy, r, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  // Heading line — world +x at yaw=0. Canvas Y is flipped relative to
  // world, so dy uses -sin(yaw); see worldToCanvas() for the same trick.
  const len = HEADING_LEN_M * view.scale;
  ctx.strokeStyle = HEADING_STROKE;
  ctx.lineWidth   = 2;
  ctx.lineCap     = "round";
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx + Math.cos(yaw) * len, cy - Math.sin(yaw) * len);
  ctx.stroke();
  ctx.restore();
}

function draw(canvas, ctx) {
  fitView(canvas);
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.width  / dpr;
  const H = canvas.height / dpr;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = "#fafafa";
  ctx.fillRect(0, 0, W, H);
  drawGrid(ctx, canvas);
  for (const s of MAP.shapes) drawShape(ctx, s);
  drawLidar(ctx);
  drawRobot(ctx);
}

function resizeCanvas(canvas, ctx) {
  // devicePixelRatio so lines stay crisp on retina / scaled displays.
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width  = Math.round(rect.width  * dpr);
  canvas.height = Math.round(rect.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw(canvas, ctx);
}

window.addEventListener("DOMContentLoaded", () => {
  canvasEl = document.getElementById("map");
  ctxRef   = canvasEl.getContext("2d");
  resizeCanvas(canvasEl, ctxRef);
  window.addEventListener("resize", () => resizeCanvas(canvasEl, ctxRef));
  connectLiveState();
});

// ---------------------------------------------------------------------------
// Live state — Server-Sent Events from web_server_node
// ---------------------------------------------------------------------------
//
// STATE mirrors web_server_node's topic cache. Read it from anywhere
// (e.g. `STATE.odom.pose.position.x`). It is populated by:
//
//   1) one `snapshot` event right after the connection opens, containing
//      every cached topic (any may still be null if no message arrived yet);
//   2) per-tick `message` events containing only the topics that changed
//      since the previous tick (default 10 Hz, server param stream_rate_hz).
//
// To react to updates, override window.onStateChange:
//
//   window.onStateChange = (changedKeys, state) => {
//       if (changedKeys.includes('odom')) drawRobot(state.odom);
//   };
//
// File:// mode has no server, so the EventSource is skipped — STATE stays
// the empty object and the canvas just shows the baked map.

const STATE = {};
window.STATE = STATE;

// Called once per SSE tick — once with the full cache (snapshot), then on
// every delta. `changedKeys` is the list of topics whose value just
// landed; `state` is the full mirrored cache. Replace the body to wire
// the UI (canvas, gauges, etc.).
function handleStateChange(changedKeys, state) {
  // Only log the topics that actually changed — STATE in full would dump
  // the base64-encoded map / costmap on every tick and flood the console.
  const delta = {};
  for (const k of changedKeys) delta[k] = state[k];
  console.log("[state]", changedKeys, delta);

  // Repaint when the pose OR the scan moves — drawLidar uses both
  // (range data from /scan_filtered, origin/yaw from /amcl_pose), so
  // either changing should refresh the canvas.
  if (canvasEl && (changedKeys.includes("amcl_pose") ||
                   changedKeys.includes("scan"))) {
    draw(canvasEl, ctxRef);
  }

  if (changedKeys.includes("dispatcher_status")) {
    updateMissionPanel(state.dispatcher_status);
  }
}

// ---------------------------------------------------------------------------
// Mission panel — driven by /dispatcher/status (auto-parsed JSON)
// ---------------------------------------------------------------------------
//
// dispatcher_status arrives wrapped as { stamp, raw, json: {...} } from
// web_server_node's _string_to_dict(). The actual payload lives under
// `.json` and looks like:
//   { state, running, index, total, current, awaiting, completed: [...],
//     waypoints: [{name, action}, ...] }
function updateMissionPanel(wrapped) {
  const panel = document.getElementById("mission-panel");
  if (!panel) return;
  if (!wrapped || !wrapped.json) return;
  const s = wrapped.json;

  // State row — text + colour class so "Running" reads blue, etc.
  const stateText = ({
    idle:     "Idle",
    running:  "Running",
    complete: "Complete",
    aborted:  "Aborted",
  })[s.state] || s.state || "—";
  panel.classList.remove("state-running", "state-complete", "state-aborted");
  if (s.state && s.state !== "idle") panel.classList.add("state-" + s.state);
  panel.querySelector("[data-field=state]").textContent    = stateText;
  panel.querySelector("[data-field=progress]").textContent =
      `${s.completed ? s.completed.length : 0} / ${s.total ?? "—"}`;
  panel.querySelector("[data-field=current]").textContent  =
      s.current || "—";

  // Waypoint list — completed = green checkmark, current = blue arrow,
  // pending = grey bullet.
  const ul = panel.querySelector("[data-field=waypoints]");
  ul.innerHTML = "";
  const done = new Set(s.completed || []);
  for (const wp of (s.waypoints || [])) {
    const li = document.createElement("li");
    let marker = "·";
    if (done.has(wp.name))           { marker = "✓"; li.classList.add("done"); }
    else if (wp.name === s.current)  { marker = "→"; li.classList.add("current"); }
    li.textContent = `${marker} ${wp.name}`;
    ul.appendChild(li);
  }
}
window.onStateChange = handleStateChange;

function connectLiveState() {
  if (location.protocol === "file:") return;   // standalone mode, no server
  const es = new EventSource("/api/stream");

  es.addEventListener("snapshot", (e) => {
    const snap = JSON.parse(e.data);
    for (const k of Object.keys(snap)) STATE[k] = snap[k];
    window.onStateChange(Object.keys(snap), STATE);
  });

  es.onmessage = (e) => {
    const delta = JSON.parse(e.data);
    const keys = Object.keys(delta);
    for (const k of keys) STATE[k] = delta[k];
    window.onStateChange(keys, STATE);
  };

  es.onerror = () => {
    // The browser auto-reconnects with backoff; nothing to do but log.
    console.warn("/api/stream disconnected, will retry");
  };
}

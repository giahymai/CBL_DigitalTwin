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

// Robot's spawn pose. Must match physical_entity.launch.py's x_pose /
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
  wireStartButton();
  wireBatteryControl();
  connectLiveState();
});

// ---------------------------------------------------------------------------
// Start Navigation button — POSTs to /api/start_navigation, which calls the
// /start_navigation ROS service on the mission dispatcher. Disabled in
// standalone (file://) mode since there's no server to talk to.
// ---------------------------------------------------------------------------

function wireStartButton() {
  const btn = document.getElementById("start-nav-btn");
  if (!btn) return;
  if (location.protocol === "file:") {
    btn.disabled = true;
    btn.title = "Open via the web_server_node — file:// has no API";
    btn.textContent = "Start (server only)";
    return;
  }
  btn.addEventListener("click", async () => {
    btn.disabled = true;
    const original = btn.textContent;
    btn.textContent = "Starting…";
    try {
      const r = await fetch("/api/start_navigation", { method: "POST" });
      const body = await r.json().catch(() => ({}));
      if (!r.ok || !body.success) {
        // Brief inline error; the dispatcher_status SSE will overwrite this
        // as soon as the next status tick arrives.
        btn.textContent = body.message
            ? `Failed: ${body.message}`
            : `Failed (HTTP ${r.status})`;
        setTimeout(() => { btn.textContent = original; btn.disabled = false; }, 2500);
      }
      // On success we leave the button disabled; the dispatcher_status
      // listener below will re-enable it when the mission ends.
    } catch (err) {
      btn.textContent = `Error: ${err.message || err}`;
      setTimeout(() => { btn.textContent = original; btn.disabled = false; }, 2500);
    }
  });
}

// Toggle the button enabled/disabled from dispatcher_status. Called from
// updateMissionPanel below so we don't duplicate the parse.
function setStartButtonRunning(running) {
  const btn = document.getElementById("start-nav-btn");
  if (!btn || location.protocol === "file:") return;
  btn.disabled = running;
  btn.textContent = running ? "Running…" : "Start Navigation";
}

// ---------------------------------------------------------------------------
// Battery override — POSTs {percent} to /api/set_battery, which publishes
// a Float64 to /battery_set_level that the battery_sim_node consumes on
// its next tick. Useful for testing the dispatcher's low-battery
// return-home trigger without waiting for the real drain.
// ---------------------------------------------------------------------------

function wireBatteryControl() {
  const btn   = document.getElementById("set-battery-btn");
  const input = document.getElementById("set-battery-input");
  if (!btn || !input) return;
  if (location.protocol === "file:") {
    btn.disabled = true; input.disabled = true; return;
  }

  const submit = async () => {
    const raw = input.value.trim();
    const pct = parseFloat(raw);
    if (raw === "" || isNaN(pct) || pct < 0 || pct > 100) {
      input.classList.add("invalid");
      setTimeout(() => input.classList.remove("invalid"), 1200);
      return;
    }
    btn.disabled = true;
    try {
      const r = await fetch("/api/set_battery", {
        method:  "POST",
        headers: { "Content-Type": "application/json" },
        body:    JSON.stringify({ percent: pct }),
      });
      if (!r.ok) {
        const body = await r.json().catch(() => ({}));
        console.warn("/api/set_battery failed:", body);
      }
    } catch (err) {
      console.warn("/api/set_battery error:", err);
    } finally {
      btn.disabled = false;
      input.value = "";
    }
  };

  btn.addEventListener("click", submit);
  input.addEventListener("keydown", (e) => {
    if (e.key === "Enter") submit();
  });
}

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

  // Repaint when the pose refreshes — drawRobot reads STATE.amcl_pose.
  if (canvasEl && changedKeys.includes("amcl_pose")) {
    draw(canvasEl, ctxRef);
  }

  if (changedKeys.includes("dispatcher_status")) {
    updateMissionPanel(state.dispatcher_status);
  }
  if (changedKeys.includes("battery")) {
    updateBatteryDisplay(state.battery);
  }
  if (changedKeys.includes("dt_status")) {
    updateActionHistory(state.dt_status);
  }
  // The Twin Status panel pulls from navigator_status, amcl_pose, and
  // dt_status, so refresh on any of them.
  if (changedKeys.includes("navigator_status") ||
      changedKeys.includes("amcl_pose") ||
      changedKeys.includes("dt_status")) {
    updateTwinStatus();
  }
}

// ---------------------------------------------------------------------------
// Twin Status panel — surfaces what the digital twin can observe about
// the robot right now: navigator state, map-frame position, dt_logger
// sync error, and which zones have been visited.
// ---------------------------------------------------------------------------

function updateTwinStatus() {
  const panel = document.getElementById("twin-panel");
  if (!panel) return;
  const nav = STATE.navigator_status && STATE.navigator_status.json;
  const dt  = STATE.dt_status        && STATE.dt_status.json;

  // Robot state — recolour the row, not just the value, so the strong
  // "returning home" colour reads at a glance.
  const stateRow = panel.querySelector('[data-row="robot-state"]');
  const stateEl  = panel.querySelector("[data-field=robot-state]");
  stateRow.classList.remove(
      "robot-idle", "robot-navigating", "robot-returning_home");
  const stateText = ({
    idle:           "Idle",
    navigating:     "Navigating",
    returning_home: "Returning home",
  })[nav && nav.state] || (nav && nav.state) || "—";
  stateEl.textContent = stateText;
  if (nav && nav.state) stateRow.classList.add("robot-" + nav.state);

  // Position — prefer AMCL (map frame). /odom drifts and the navigator
  // status carries odom too, so this is the authoritative one.
  const pose = (STATE.amcl_pose && STATE.amcl_pose.pose) || null;
  panel.querySelector("[data-field=robot-position]").textContent =
      pose
          ? `(${pose.position.x.toFixed(2)}, ${pose.position.y.toFixed(2)}) m`
          : "—";

  // Current goal — what the navigator is driving toward right now.
  panel.querySelector("[data-field=robot-goal]").textContent =
      (nav && nav.current) ? nav.current : "—";

  // dt_logger output: sync error and the zones it has logged.
  const syncErr = dt && typeof dt.sync_error_m === "number"
      ? dt.sync_error_m
      : null;
  panel.querySelector("[data-field=sync-error]").textContent =
      syncErr == null ? "—" : `${syncErr.toFixed(2)} m`;

  // Show zones as their last char (A/B/C/D) for compactness, plus a
  // count so the meaning stays clear with zero visited.
  const zones = (dt && Array.isArray(dt.zones_covered))
      ? dt.zones_covered : [];
  const initials = zones.map(n => String(n).slice(-1)).join(", ");
  panel.querySelector("[data-field=zones-visited]").textContent =
      zones.length === 0
          ? "0"
          : `${zones.length}  (${initials})`;
}

// ---------------------------------------------------------------------------
// Action history — driven by /dt/status (auto-parsed JSON). The payload
// carries spray/fertilize counts plus a `recent_actions` array of the
// last ~20 events; each entry is { action, zone, logged_at }.
// ---------------------------------------------------------------------------

function updateActionHistory(wrapped) {
  const panel = document.getElementById("actions-panel");
  if (!panel) return;
  if (!wrapped || !wrapped.json) return;
  const s = wrapped.json;

  panel.querySelector("[data-field=spray-count]").textContent =
      String(s.spray_actions ?? 0);
  panel.querySelector("[data-field=fert-count]").textContent  =
      String(s.fertilize_actions ?? 0);

  const ol = panel.querySelector("[data-field=action-list]");
  ol.innerHTML = "";

  const actions = Array.isArray(s.recent_actions) ? s.recent_actions : [];
  if (actions.length === 0) {
    const li = document.createElement("li");
    li.className = "empty";
    li.textContent = "No actions yet.";
    ol.appendChild(li);
    return;
  }

  // Most-recent first reads better in a feed.
  for (let i = actions.length - 1; i >= 0; i--) {
    const a = actions[i];
    const li = document.createElement("li");
    if (a.action === "spray" || a.action === "fertilize") {
      li.classList.add(a.action);
    }
    // Build with DOM nodes (not innerHTML) so a malicious zone name
    // can't inject markup.
    const action = document.createElement("b");
    action.textContent = String(a.action || "?").toUpperCase();
    const zone = document.createElement("span");
    zone.className = "zone";
    zone.textContent = String(a.zone || "");
    const time = document.createElement("span");
    time.className = "time";
    time.textContent = formatActionTime(a.logged_at);
    li.append(action, zone, time);
    ol.appendChild(li);
  }
}

function formatActionTime(iso) {
  // dt_logger writes datetime.now().isoformat() — e.g. "2026-06-13T12:34:56.7"
  if (!iso) return "—";
  const m = /T(\d{2}:\d{2}:\d{2})/.exec(String(iso));
  return m ? m[1] : String(iso).slice(11, 19) || "—";
}

// ---------------------------------------------------------------------------
// Battery row — driven by /battery_state (BatteryState dict). The web
// server caches msg.percentage as 0..1; we render it as 0..100 % and
// colour-code by the same thresholds the navigator uses for return-home.
// ---------------------------------------------------------------------------

function updateBatteryDisplay(battery) {
  const panel = document.getElementById("mission-panel");
  if (!panel) return;
  const row = panel.querySelector('[data-row="battery"]');
  const el  = panel.querySelector("[data-field=battery]");
  if (!row || !el) return;
  // Clear previous level class regardless of validity.
  row.classList.remove("battery-good", "battery-warn", "battery-low");

  if (!battery || battery.percentage == null) {
    el.textContent = "—";
    return;
  }
  const pct = battery.percentage * 100;
  const v   = battery.voltage;
  el.textContent = (v != null && !isNaN(v))
      ? `${pct.toFixed(0)}% (${v.toFixed(1)} V)`
      : `${pct.toFixed(0)}%`;
  // Match navigator_node's 20 % return-home threshold; "warn" is just a
  // visual heads-up before that kicks in.
  if      (pct < 20) row.classList.add("battery-low");
  else if (pct < 50) row.classList.add("battery-warn");
  else               row.classList.add("battery-good");
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
  setStartButtonRunning(!!s.running);

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

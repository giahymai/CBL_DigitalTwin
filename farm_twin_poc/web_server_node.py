#!/usr/bin/env python3
"""
web_server_node.py — ROS subscriber + built-in HTTP server for the web UI
========================================================================
Farm Twin PoC | Team 5 Terra Minds

Subscribes to every topic gazebo_nav2_demo exposes and serves both:
  - the latest cached message of each topic, as JSON, under /api/state[...]
  - static files (index.html / style.css / app.js) from the package's `web/`
    directory, at /

Uses only the Python standard library (http.server, threading, json, base64).
No Flask / FastAPI / rosbridge.

----------------------------------------------------------------------------
RUN
----------------------------------------------------------------------------
  colcon build --packages-select farm_twin_poc && source install/setup.bash
  ros2 run farm_twin_poc web_server_node
  # then open http://localhost:8080/ in a browser

  # override host/port:
  ros2 run farm_twin_poc web_server_node --ros-args \\
      -p host:=0.0.0.0 -p port:=8080

----------------------------------------------------------------------------
ENDPOINTS
----------------------------------------------------------------------------
  GET /                  -> web/index.html
  GET /style.css         -> web/style.css
  GET /app.js            -> web/app.js
  GET /<anything-else>   -> web/<anything-else>   (404 if missing)

  GET /api/state         -> JSON dict of every cached topic (one-shot)
  GET /api/state/<name>  -> JSON for one topic (e.g. /api/state/odom)
  GET /api/topics        -> list of cached topic names
  GET /api/stream        -> Server-Sent Events: live push of state changes

The push endpoint sends:
  - first event `snapshot`         with the full cached state
  - then per-tick `data`           events containing only the topics that
                                   changed since the last tick (10 Hz)
  - heartbeat comments every 15 s  so reverse-proxies don't time out

Browser side, the contract is one line:
    new EventSource('/api/stream').onmessage = (e) =>
        Object.assign(STATE, JSON.parse(e.data));

`/api/state` responses are cache-busted with `Cache-Control: no-store`; the
static files use a short cache so you can hot-reload while developing.

----------------------------------------------------------------------------
WHAT'S IN THE CACHE
----------------------------------------------------------------------------
Each key holds either None (no message yet) or a dict shaped like the ROS
message, with stamps as float seconds and quaternions as {x,y,z,w}. Yaw is
added under `pose.yaw` for convenience.

  odom              nav_msgs/Odometry
  amcl_pose         geometry_msgs/PoseWithCovarianceStamped
  scan              sensor_msgs/LaserScan      (decimated, see scan_decimate)
  imu               sensor_msgs/Imu
  battery           sensor_msgs/BatteryState
  cmd_vel           geometry_msgs/TwistStamped
  goal_pose         geometry_msgs/PoseStamped
  plan              nav_msgs/Path
  local_plan        nav_msgs/Path
  map               nav_msgs/OccupancyGrid     (data base64-encoded)
  global_costmap    nav_msgs/OccupancyGrid     (data base64-encoded)
  local_costmap     nav_msgs/OccupancyGrid     (data base64-encoded)
  farm_action       std_msgs/String            (auto-parsed if JSON)
  navigator_status  std_msgs/String            (auto-parsed if JSON)
  dt_status         std_msgs/String            (auto-parsed if JSON)
  dispatcher_status std_msgs/String            (JSON: state/index/total/waypoints/...)

OccupancyGrid `data` is base64-encoded so it survives JSON without bloating
the payload. Decode in JS with:
  const bytes = Uint8Array.from(atob(s), c => c.charCodeAt(0));
  const grid  = new Int8Array(bytes.buffer);
"""

from __future__ import annotations

import base64
import copy
import json
import math
import os
import queue
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    TwistStamped,
)
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# helpers — convert ROS messages to JSON-friendly dicts
# ---------------------------------------------------------------------------

def _stamp(msg_or_header) -> float:
    h = getattr(msg_or_header, 'header', msg_or_header)
    s = getattr(h, 'stamp', None)
    if s is None:
        return 0.0
    return s.sec + s.nanosec * 1e-9


def _yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _point(p):
    return {'x': p.x, 'y': p.y, 'z': p.z}


def _quat(q):
    return {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}


def _pose(p):
    return {
        'position':    _point(p.position),
        'orientation': _quat(p.orientation),
        'yaw':         _yaw_from_quat(p.orientation),
    }


def _twist(t):
    return {
        'linear':  {'x': t.linear.x,  'y': t.linear.y,  'z': t.linear.z},
        'angular': {'x': t.angular.x, 'y': t.angular.y, 'z': t.angular.z},
    }


def _odom_to_dict(msg: Odometry) -> dict:
    return {
        'stamp':       _stamp(msg),
        'frame_id':    msg.header.frame_id,
        'child_frame': msg.child_frame_id,
        'pose':        _pose(msg.pose.pose),
        'twist':       _twist(msg.twist.twist),
    }


def _pose_cov_to_dict(msg: PoseWithCovarianceStamped) -> dict:
    return {
        'stamp':      _stamp(msg),
        'frame_id':   msg.header.frame_id,
        'pose':       _pose(msg.pose.pose),
        'covariance': list(msg.pose.covariance),
    }


def _pose_stamped_to_dict(msg: PoseStamped) -> dict:
    return {
        'stamp':    _stamp(msg),
        'frame_id': msg.header.frame_id,
        'pose':     _pose(msg.pose),
    }


def _twist_stamped_to_dict(msg: TwistStamped) -> dict:
    return {
        'stamp':    _stamp(msg),
        'frame_id': msg.header.frame_id,
        'twist':    _twist(msg.twist),
    }


def _scan_to_dict(msg: LaserScan, decimate: int) -> dict:
    decimate = max(1, int(decimate))
    return {
        'stamp':           _stamp(msg),
        'frame_id':        msg.header.frame_id,
        'angle_min':       msg.angle_min,
        'angle_max':       msg.angle_max,
        'angle_increment': msg.angle_increment * decimate,
        'range_min':       msg.range_min,
        'range_max':       msg.range_max,
        # NaN/Inf are not JSON-legal; allow_nan=False would raise on them.
        'ranges': [
            (r if math.isfinite(r) else None) for r in msg.ranges[::decimate]
        ],
    }


def _imu_to_dict(msg: Imu) -> dict:
    return {
        'stamp':               _stamp(msg),
        'frame_id':            msg.header.frame_id,
        'orientation':         _quat(msg.orientation),
        'yaw':                 _yaw_from_quat(msg.orientation),
        'angular_velocity':    {'x': msg.angular_velocity.x,
                                'y': msg.angular_velocity.y,
                                'z': msg.angular_velocity.z},
        'linear_acceleration': {'x': msg.linear_acceleration.x,
                                'y': msg.linear_acceleration.y,
                                'z': msg.linear_acceleration.z},
    }


def _battery_to_dict(msg: BatteryState) -> dict:
    return {
        'stamp':      _stamp(msg),
        'voltage':    msg.voltage,
        'current':    msg.current,
        'charge':     msg.charge,
        'percentage': msg.percentage,
        'present':    msg.present,
    }


def _path_to_dict(msg: Path) -> dict:
    return {
        'stamp':    _stamp(msg),
        'frame_id': msg.header.frame_id,
        'poses': [
            {'stamp': _stamp(p), 'pose': _pose(p.pose)} for p in msg.poses
        ],
    }


def _grid_to_dict(msg: OccupancyGrid) -> dict:
    # OccupancyGrid.data is int8[] (-1..100), up to ~150k cells. base64 the
    # raw bytes — ~4x smaller and ~10x faster to parse than a JSON list.
    raw = bytes(((v + 256) & 0xFF) for v in msg.data)
    return {
        'stamp':    _stamp(msg),
        'frame_id': msg.header.frame_id,
        'info': {
            'resolution': msg.info.resolution,
            'width':      msg.info.width,
            'height':     msg.info.height,
            'origin':     _pose(msg.info.origin),
        },
        'data_b64': base64.b64encode(raw).decode('ascii'),
    }


def _string_to_dict(msg: String, now: float) -> dict:
    # navigator_node and dt_logger_node publish JSON in std_msgs/String — try
    # to parse it for the web side, fall back to raw text.
    try:
        return {'stamp': now, 'json': json.loads(msg.data), 'raw': msg.data}
    except json.JSONDecodeError:
        return {'stamp': now, 'raw': msg.data}


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class WebServerNode(Node):
    """Subscribes to demo topics + serves them over HTTP."""

    # transient_local QoS so latched publishers (the map, costmaps, global
    # plan) are actually received. Default sensor QoS would miss them.
    _LATCHED_QOS = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
    )

    def __init__(self):
        super().__init__('web_server_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('scan_decimate', 4)
        # 30 Hz matches /odom's native publish rate from the Gazebo bridge,
        # so pushing higher gains nothing (no new data between ticks).
        # Lower if you see CPU pressure or bandwidth issues over the link.
        self.declare_parameter('stream_rate_hz', 30.0)

        self._scan_decimate = int(
            self.get_parameter('scan_decimate').value)
        stream_period = 1.0 / max(0.1, float(
            self.get_parameter('stream_rate_hz').value))

        # SSE plumbing — one queue per live /api/stream connection. Topic
        # callbacks mark keys dirty in `_dirty`; a periodic timer drains
        # the set, builds a delta, and fans it out to every queue.
        self._sse_lock:    threading.Lock         = threading.Lock()
        self._sse_clients: list[queue.Queue[str]] = []
        self._dirty:       set[str]               = set()

        self._lock  = threading.Lock()
        self._state: dict[str, Optional[dict]] = {
            'odom':             None,
            'amcl_pose':        None,
            'scan':             None,
            'imu':              None,
            'battery':          None,
            'cmd_vel':          None,
            'goal_pose':        None,
            'plan':             None,
            'local_plan':       None,
            'map':              None,
            'global_costmap':   None,
            'local_costmap':    None,
            'farm_action':      None,
            'navigator_status':  None,
            'dt_status':         None,
            'dispatcher_status': None,
        }

        # Sensors / odometry.
        self.create_subscription(Odometry,
            '/odom', self._cb_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped,
            '/amcl_pose', self._cb_amcl, 10)
        self.create_subscription(LaserScan,
            '/scan', self._cb_scan, 10)
        self.create_subscription(Imu,
            '/imu', self._cb_imu, 10)
        self.create_subscription(BatteryState,
            '/battery_state', self._cb_battery, 10)

        # Nav2 command + goal + plans.
        self.create_subscription(TwistStamped,
            '/cmd_vel', self._cb_cmd_vel, 10)
        self.create_subscription(PoseStamped,
            '/goal_pose', self._cb_goal, 10)
        self.create_subscription(Path,
            '/plan', self._cb_plan, 10)
        self.create_subscription(Path,
            '/local_plan', self._cb_local_plan, 10)

        # Latched maps & costmaps.
        self.create_subscription(OccupancyGrid,
            '/map', self._cb_map, self._LATCHED_QOS)
        self.create_subscription(OccupancyGrid,
            '/global_costmap/costmap',
            self._cb_global_costmap, self._LATCHED_QOS)
        self.create_subscription(OccupancyGrid,
            '/local_costmap/costmap',
            self._cb_local_costmap, self._LATCHED_QOS)

        # Farm-specific topics.
        self.create_subscription(String,
            '/farm_action', self._cb_farm_action, 10)
        self.create_subscription(String,
            '/navigator/status', self._cb_nav_status, 10)
        self.create_subscription(String,
            '/dt/status', self._cb_dt_status, 10)
        self.create_subscription(String,
            '/dispatcher/status', self._cb_dispatcher_status, 10)

        # Static web root — created by setup.py from <pkg_root>/web/.
        self._web_root = os.path.join(
            get_package_share_directory('farm_twin_poc'), 'web')

        # HTTP server in a background daemon thread; rclpy.spin keeps the
        # main thread, so the process exits cleanly on Ctrl-C.
        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)
        self._httpd = ThreadingHTTPServer(
            (host, port), _make_handler(self, self._web_root))
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

        # SSE broadcast tick — coalesces N callbacks per period into one
        # delta event so a 60 Hz /odom topic doesn't become 60 events/s.
        self.create_timer(stream_period, self._sse_broadcast)

        self.get_logger().info(
            f'web_server_node up — http://{host}:{port}/  '
            f'(static root: {self._web_root}, '
            f'SSE rate: {1.0/stream_period:.1f} Hz)')

    # -- public API used by the HTTP handler --------------------------------

    def get_state(self) -> dict[str, Any]:
        with self._lock:
            return copy.deepcopy(self._state)

    def get(self, name: str) -> Optional[dict]:
        with self._lock:
            return copy.deepcopy(self._state.get(name))

    def topics(self) -> list[str]:
        return list(self._state.keys())

    def shutdown(self) -> None:
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:
            pass

    # -- SSE pub/sub API used by the HTTP handler ---------------------------

    def sse_subscribe(self) -> 'queue.Queue[str]':
        """Register a fresh client queue. Caller must call sse_unsubscribe
        on disconnect. Bounded so a stalled browser tab can't grow memory
        without limit — on overflow we drop the oldest payload."""
        q: queue.Queue[str] = queue.Queue(maxsize=100)
        with self._sse_lock:
            self._sse_clients.append(q)
        return q

    def sse_unsubscribe(self, q: 'queue.Queue[str]') -> None:
        with self._sse_lock:
            if q in self._sse_clients:
                self._sse_clients.remove(q)

    def _sse_broadcast(self) -> None:
        with self._sse_lock:
            if not self._dirty or not self._sse_clients:
                self._dirty.clear()
                return
            dirty = list(self._dirty)
            self._dirty.clear()
            clients = list(self._sse_clients)
        with self._lock:
            delta = {k: copy.deepcopy(self._state[k]) for k in dirty}
        payload = json.dumps(delta, allow_nan=False)
        # Fan out to every client; on a full queue, drop the oldest and
        # retry once. If the client is so slow we still can't enqueue,
        # let the next tick try again.
        for q in clients:
            try:
                q.put_nowait(payload)
            except queue.Full:
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    q.put_nowait(payload)
                except queue.Full:
                    pass

    # -- callbacks ----------------------------------------------------------

    def _store(self, key: str, value: dict) -> None:
        with self._lock:
            self._state[key] = value
        # Marked AFTER the state update so the broadcast snapshot is the
        # new value, never the previous one.
        with self._sse_lock:
            self._dirty.add(key)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_odom(self, msg):
        self._store('odom', _odom_to_dict(msg))

    def _cb_amcl(self, msg):
        self._store('amcl_pose', _pose_cov_to_dict(msg))

    def _cb_scan(self, msg):
        self._store('scan', _scan_to_dict(msg, self._scan_decimate))

    def _cb_imu(self, msg):
        self._store('imu', _imu_to_dict(msg))

    def _cb_battery(self, msg):
        self._store('battery', _battery_to_dict(msg))

    def _cb_cmd_vel(self, msg):
        self._store('cmd_vel', _twist_stamped_to_dict(msg))

    def _cb_goal(self, msg):
        self._store('goal_pose', _pose_stamped_to_dict(msg))

    def _cb_plan(self, msg):
        self._store('plan', _path_to_dict(msg))

    def _cb_local_plan(self, msg):
        self._store('local_plan', _path_to_dict(msg))

    def _cb_map(self, msg):
        self._store('map', _grid_to_dict(msg))

    def _cb_global_costmap(self, msg):
        self._store('global_costmap', _grid_to_dict(msg))

    def _cb_local_costmap(self, msg):
        self._store('local_costmap', _grid_to_dict(msg))

    def _cb_farm_action(self, msg):
        self._store('farm_action', _string_to_dict(msg, self._now()))

    def _cb_nav_status(self, msg):
        self._store('navigator_status', _string_to_dict(msg, self._now()))

    def _cb_dt_status(self, msg):
        self._store('dt_status', _string_to_dict(msg, self._now()))

    def _cb_dispatcher_status(self, msg):
        self._store('dispatcher_status', _string_to_dict(msg, self._now()))


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

# index.html is served for "/"; everything else is looked up by name. Listed
# here so the handler logs a helpful hint if a file is missing on disk.
_DEFAULT_FILE = 'index.html'

# minimal content-type table — enough for the three stub files plus a few
# extras you might add later. stdlib `mimetypes` would also work but this is
# explicit and dependency-free.
_MIME = {
    '.html': 'text/html; charset=utf-8',
    '.css':  'text/css; charset=utf-8',
    '.js':   'application/javascript; charset=utf-8',
    '.json': 'application/json; charset=utf-8',
    '.png':  'image/png',
    '.jpg':  'image/jpeg',
    '.svg':  'image/svg+xml',
    '.ico':  'image/x-icon',
}


def _make_handler(node: 'WebServerNode', web_root: str):
    """Build a request handler class bound to this node + static root."""

    class Handler(BaseHTTPRequestHandler):
        # Silence default per-request stderr spam; route through ROS logger.
        def log_message(self, fmt, *args):
            node.get_logger().debug('http: ' + (fmt % args))

        # -- routing ----------------------------------------------------

        def do_GET(self):
            path = self.path.split('?', 1)[0]   # strip query string
            if path == '/api/state':
                return self._send_json(node.get_state())
            if path == '/api/topics':
                return self._send_json(node.topics())
            if path == '/api/stream':
                return self._send_sse()
            if path.startswith('/api/state/'):
                name = path[len('/api/state/'):]
                data = node.get(name)
                if data is None and name not in node.topics():
                    return self._send_error(
                        HTTPStatus.NOT_FOUND, f'unknown topic: {name}')
                return self._send_json(data)
            return self._send_static(path)

        # -- helpers ----------------------------------------------------

        def _send_json(self, obj):
            body = json.dumps(obj, allow_nan=False).encode('utf-8')
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            # Permissive CORS — the web UI is probably hosted from this same
            # server, but allow simple cross-origin probes for dev too.
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(body)

        def _send_sse(self):
            # Long-lived response. ThreadingHTTPServer gives each connection
            # its own thread, so blocking here only blocks this client.
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.send_header('X-Accel-Buffering', 'no')   # disable nginx buffering
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            try:
                # First payload is the full snapshot so a freshly-loaded page
                # has every topic even if nothing has changed since it
                # connected. The client distinguishes it via `event:`.
                snap = json.dumps(node.get_state(), allow_nan=False)
                self.wfile.write(
                    f'event: snapshot\ndata: {snap}\n\n'.encode('utf-8'))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

            q = node.sse_subscribe()
            try:
                while True:
                    # 15 s heartbeat: SSE comments (lines starting with `:`)
                    # are no-ops on the client but keep the TCP connection
                    # alive through proxies / load balancers.
                    try:
                        payload = q.get(timeout=15.0)
                        self.wfile.write(
                            f'data: {payload}\n\n'.encode('utf-8'))
                    except queue.Empty:
                        self.wfile.write(b': heartbeat\n\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ValueError):
                # Client closed the tab / network died.
                pass
            finally:
                node.sse_unsubscribe(q)

        def _send_static(self, path: str):
            rel = path.lstrip('/') or _DEFAULT_FILE
            full = os.path.realpath(os.path.join(web_root, rel))
            # Block path traversal: resolved path must stay inside web_root.
            if not full.startswith(os.path.realpath(web_root) + os.sep) \
                    and full != os.path.realpath(web_root):
                return self._send_error(HTTPStatus.FORBIDDEN, 'forbidden')
            if not os.path.isfile(full):
                return self._send_error(
                    HTTPStatus.NOT_FOUND, f'not found: /{rel}')
            ext = os.path.splitext(full)[1].lower()
            ctype = _MIME.get(ext, 'application/octet-stream')
            try:
                with open(full, 'rb') as f:
                    body = f.read()
            except OSError as e:
                return self._send_error(
                    HTTPStatus.INTERNAL_SERVER_ERROR, str(e))
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            # Short cache so you can edit web/* and reload without restarting.
            self.send_header('Cache-Control', 'public, max-age=2')
            self.end_headers()
            self.wfile.write(body)

        def _send_error(self, status: HTTPStatus, msg: str):
            body = json.dumps({'error': msg}).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


# ---------------------------------------------------------------------------
# entry point
# ---------------------------------------------------------------------------

def main():
    rclpy.init()
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# Farm Twin — Proof of Concept (Option B)

**Team 5 Terra Minds | SDG 2: Zero Hunger | Course 2IRR10 | TU/e**

---

## What Is This PoC?

The Farm Twin technical solution proposes autonomous robots for maize
smallholders in Vihiga & Kakamega Counties, Kenya — a **weeding/sprayer** robot
and a **fertilizer** robot, governed by a Digital Twin that mirrors their state
and lets an operator monitor and command them.

This PoC implements the assignment's **Option B (no physical robot)**: the whole
twin lives in simulation, but it is still built from **two distinct entities**:

| Role | In this PoC |
|---|---|
| **Physical Entity (PE)** — the "real-world stand-in", source of truth | **TurtleBot3 in Gazebo** (farm world) driven by **Nav2**, plus an internal-state layer (battery, sensor health, motor, mode) and a command gate |
| **Digital Entity (DE)** — the second representation that mirrors & controls it | **A web dashboard** (rosbridge + browser) that visualises the PE, computes metrics, and sends commands and fault injections back |

ROS 2 (Jazzy) connects the two sides.

---

## What Does This PoC Prove? (the three required DT usages)

### ① Bidirectional Communication (pub/sub both ways)
- **PE → DE**: Gazebo streams `/scan`/`/scan_filtered`, `/odom`, and the
  internal state `/pe/state`; the dashboard subscribes and visualises them.
- **DE → PE**: the dashboard publishes `/de/cmd_vel` (manual drive),
  `/de/command` (fault injection), and calls the navigation services — the PE
  acts on them.

**Evidence:** open the dashboard, drive the robot with the on-screen pad, and
watch the scan/pose update live; or:
```bash
ros2 topic echo /pe/state --once     # PE → DE : internal state
ros2 topic echo /de/cmd_vel          # DE → PE : commands from the dashboard
```

### ② State Synchronisation (states, not just commands)
`pe_state_node` maintains the PE's **internal state** and publishes it on
`/pe/state` (≈4 Hz). The dashboard mirrors it in real time: **battery %**,
**LiDAR health** (OK/DEGRADED/FAILED), **motor status**, **mode of operation**
(idle/navigating/returning_home/teleop/SAFE_STOP), and **localization quality**.
`dt_logger_node` republishes a consolidated `/dt/status` with a `link_age_s`
freshness figure proving the mirror is near-real-time.

**Fault injection (DE → PE → DE):** click a fault button on the dashboard →
`pe_state_node` changes the PE's behaviour (e.g. a failed LiDAR empties
`/scan_filtered` and emergency-stops the robot, a low battery triggers
return-home) → the new state propagates straight back to the dashboard.

**Evidence:**
```bash
ros2 topic echo /pe/state --once
ros2 service call /get_dt_status std_srvs/srv/Trigger
ros2 topic pub --once /de/command std_msgs/msg/String '{data: "{\"target\":\"lidar\",\"action\":\"fail\"}"}'
# -> lidar_health goes FAILED, emergency_stop true, robot stops, dashboard turns red
```

### ③ Environmental Interaction (propagated across the twin)
- **Obstacle detection & avoidance:** Nav2 builds a costmap from
  `/scan_filtered` and plans a collision-free path to each farm zone; drop a new
  obstacle into the Gazebo world and the robot re-plans around it — and the
  dashboard's LiDAR view shows the same obstacle.
- **Sensing change via fault:** a DEGRADED/FAILED LiDAR (injected from the DE)
  changes what Nav2 senses and what the dashboard shows — the environment event
  is reflected on **both** sides, not local to one.
- **Zone actions:** when the robot reaches a farm zone, `zone_monitor_node`
  fires `/farm_action` (spray/fertilize), `dt_logger_node` logs it, and the
  dashboard's action counters increment.

**Evidence:**
```bash
ros2 topic echo /farm_action
ros2 service call /get_dt_log std_srvs/srv/Trigger
```

---

## System Architecture

```
PHYSICAL ENTITY (Gazebo, source of truth)              DIGITAL ENTITY (browser)
┌───────────────────────────────────────┐             ┌────────────────────────┐
│ Gazebo TB3  ── /scan ──► pe_state_node │             │      Web dashboard     │
│             ── /odom ──────────────┐   │   /pe/state │  · battery / sensors   │
│ Nav2 (costmap on /scan_filtered)   │   │ ──────────► │  · mode / localization │
│   controller→smoother→collision_monitor│   /scan_*   │  · LiDAR view + metric │
│        │ /cmd_vel_pe                │   │ ──────────► │  · farm action log     │
│        ▼                            │   │   /odom     │                        │
│   pe_state_node ── /cmd_vel ──► robot   │ ◄────────── │  /de/cmd_vel (drive)   │
│   (command gate + fault + state)   │   │ ◄────────── │  /de/command (faults)  │
│ navigator_node · zone_monitor_node │   │  services   │  start/stop/return     │
│ dt_logger_node ── /dt/status ──────┼───┼──── rosbridge_websocket :9090 ───────┘
└───────────────────────────────────────┘
```

Everything starts from **one launch file** — there is only one run-time stack.

---

## Nodes, Topics, Services

| Concept | Where |
|---|---|
| **PE nodes** | `pe_state_node` (internal state + `/cmd_vel` gate + LiDAR fault), `navigator_node`/`nav2_navigator` (Nav2 zone tour + return-home), `zone_monitor_node` (farm-zone `/farm_action`), `nav2` stack (planner/controller/AMCL) |
| **DE** | web dashboard (`web/index.html` via rosbridge), `dt_logger_node` (DE aggregator → `/dt/status`) |
| **Topics** | `/scan`, `/scan_filtered`, `/odom`, `/cmd_vel`, `/cmd_vel_pe`, `/de/cmd_vel`, `/de/command`, `/pe/state`, `/battery_state`, `/farm_action`, `/dt/status`, `/navigator/status`, `/goal_pose` |
| **Services** | `/start_navigation`, `/return_home`, `/stop_navigation`, `/nav_status`, `/get_zone_status`, `/get_dt_status`, `/get_dt_log` |

---

## Package Structure

```
farm_twin_poc/
├── farm_twin_poc/
│   ├── pe_state_node.py     Physical Entity core: internal state, /cmd_vel gate, LiDAR fault
│   ├── navigator_node.py    Nav2 zone tour + return-home (executable: nav2_navigator)
│   ├── zone_monitor_node.py Farm-zone detection → /farm_action
│   └── dt_logger_node.py    Digital Entity aggregator → /dt/status
├── web/
│   └── index.html           The Digital Entity dashboard (roslibjs)
├── launch/
│   └── farm_twin.launch.py  Whole system in one command (PE + Nav2 + DE)
├── scripts/
│   └── world_to_map.py      Regenerate the Nav2 map from the world (only if walls change)
├── worlds/
│   └── lab_world.sdf         Farm world with flat coloured zone tiles
├── maps/
│   └── lab_map.yaml / .pgm   Nav2 map (matches the world 1:1)
└── config/
    └── nav2_sim.yaml         Nav2 params (scan = /scan_filtered, cmd out = /cmd_vel_pe)
```

---

## Farm Zones

4 zones marked as flat coloured discs (radius 0.35 m) in Gazebo. Red = spray
(Striga herbicide), green = fertilize (NPK). Robot spawns top-left at `(3, 3)`.

```
TOP-LEFT  spray_zone_C   (3.5, 2.7) [red]    spray_zone_A   (0.5, 2.7) [red]    TOP-RIGHT
BOTTOM-LEFT fertilize_zone_B (3.5, 0.7) [green]  fertilize_zone_D (0.5, 0.7) [green]  BOTTOM-RIGHT
```
Tour order: C → A → D → B. Discs are deliberately flat (~2 cm) so they sit under
the LiDAR plane — the robot drives over them and `zone_monitor_node` fires the
action. To move a zone, edit it in **all three** places: `zone_monitor_node.py`
`FARM_ZONES`, `navigator_node.py` `WAYPOINTS`, and the disc `<pose>` in
`worlds/lab_world.sdf`.

---

## Setup & Run (Gazebo, at home / in Docker)

### 0. Start the Docker container (at home, every session)
1. Open **Docker Desktop** on Windows.
2. Open **Ubuntu** from the Start Menu (not PowerShell).
3. Start the container (`--net=host` so the browser on Windows reaches rosbridge,
   and ports 8080/9090 are shared via host networking):
```bash
docker run --rm -it --name turtlebot3_container --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/c2irr10/turtlebot3_ws:/ws \
  --user $(id -u):$(id -g) turtlebot3_ws bash
```
**Every additional terminal** attaches to the same container, then sources (step 1):
```bash
docker exec -it turtlebot3_container bash
```
> `Error: No such container` → the container isn't running. Repeat step 3.

### 1. Sourcing (every new terminal)
```bash
cd /ws                                   # or ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

### 2. One-time dependencies
```bash
sudo apt update && sudo apt install -y ros-jazzy-rosbridge-suite   # web dashboard bridge
cd /ws && colcon build --packages-select farm_twin_poc && source install/setup.bash
ros2 pkg executables farm_twin_poc
# Expected: dt_logger_node, nav2_navigator, pe_state_node, zone_monitor_node
```

### 3. Launch the whole twin
```bash
ros2 launch farm_twin_poc farm_twin.launch.py
# GPU-less machine (Docker-in-WSL): run Gazebo headless
ros2 launch farm_twin_poc farm_twin.launch.py headless:=true
```

> **No GPU? Use `headless:=true`.** Gazebo needs a GPU to render the 3D window
> and the LiDAR. On a GPU-less box the sim clock stutters and Nav2 freezes;
> headless drops the 3D window (you still get RViz + the web dashboard).

### 4. Open the Digital Entity dashboard
Wait for the log to show **"Nav2 is active"**, then open:
```
http://localhost:8080
```
The dashboard auto-connects to `ws://localhost:9090` (rosbridge). If you open it
from another machine, type that host's name/IP in the `ws://___` box in the
header.

---

## Demo Script — the three requirements

**① Bidirectional + DE→PE control**
- On the dashboard, hold the drive pad (or W/A/S/D) → the robot moves in Gazebo,
  and `/odom`/scan update live on the dashboard.

**② State synchronisation + fault injection**
- Click **Start tour** → mode shows `navigating`, battery slowly drains, all live
  on the dashboard.
- Click **LiDAR fail** → `/scan_filtered` empties, the dashboard sensor turns
  red, mode → `SAFE_STOP`, the robot halts. Click **LiDAR ok** → it resumes.
- Click **Battery → 15%** → the navigator cancels the tour and returns home
  (low-battery auto-return), reflected on the dashboard.

**③ Environmental interaction**
- During the tour the robot follows a Nav2-planned path around the walls; add an
  obstacle in Gazebo and it re-plans, and the dashboard LiDAR view shows it.
- Reaching each zone fires a spray/fertilize action — the dashboard counters and
  the log update:
```bash
ros2 topic echo /farm_action
ros2 service call /get_dt_log std_srvs/srv/Trigger
```

---

## Monitoring commands
```bash
ros2 topic echo /pe/state            # PE internal state (battery, sensors, mode)
ros2 topic echo /dt/status           # DE aggregate + link freshness
ros2 topic echo /farm_action         # zone spray/fertilize events
ros2 topic echo /navigator/status    # tour progress
ros2 service call /get_dt_status  std_srvs/srv/Trigger
ros2 service call /get_dt_log     std_srvs/srv/Trigger
ros2 service call /start_navigation std_srvs/srv/Trigger
ros2 service call /return_home      std_srvs/srv/Trigger
ros2 service call /stop_navigation  std_srvs/srv/Trigger
```

---

## Troubleshooting

**Dashboard says "disconnected":** rosbridge isn't running or the port/host is
wrong. Check `ros2 node list` shows `/rosbridge_websocket`; confirm the host in
the `ws://___` box; ensure `ros-jazzy-rosbridge-suite` is installed.

**Dashboard loads but nothing updates:** the page needs internet for the roslibjs
CDN. Check the browser console; for an offline lab, download `roslib.min.js` into
`web/` and point the `<script>` tag at it.

**Robot won't move / RViz shows nothing / Nav2 timeouts:** Nav2 is waiting for
`/scan_filtered`. Confirm `pe_state_node` is up (`ros2 topic hz /scan_filtered`).
On a GPU-less machine use `headless:=true`.

**Robot stays stopped:** check `/pe/state` — if `emergency_stop` is true, a fault
is active. Click **Reset all** on the dashboard (or
`ros2 topic pub --once /de/command std_msgs/msg/String '{data: "{\"target\":\"system\",\"action\":\"reset\"}"}'`).

**Zone actions not triggering:** the robot must come within 0.35 m of a zone
centre; `zone_monitor_node` detects in the **map** frame via TF.

**Build error "failed to create symbolic link":**
```bash
cd /ws && rm -rf build/ install/ log/
colcon build --packages-select farm_twin_poc && source install/setup.bash
```

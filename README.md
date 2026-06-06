# Farm Twin — Proof of Concept

**Team 5 Terra Minds | SDG 2: Zero Hunger | Course 2IRR10 | TU/e**

---

## What Is This PoC?

The Farm Twin technical solution proposes two autonomous robots for Maize
Smallholders in Vihiga & Kakamega Counties, Kenya:

- **Weeding/Sprayer Robot** — detects Striga-infested zones, applies herbicide
- **Fertilizer Robot** — delivers variable-rate NPK per soil sensor data

Both robots are governed by a **Digital Entity** (DE) that continuously
receives sensor data from the Physical Entity (PE), makes decisions, and
sends commands back.

This PoC uses a **single TurtleBot3 Burger** to prove the core Digital Twin
mechanisms are feasible before building the full system.

---

## What Does This PoC Prove?

### ① Bi-directional Communication
- PE → DE: `/scan` (LiDAR) and `/odom` (position) flow from robot to nodes
- DE → PE: `twin_safety_node` sends safe `/cmd_vel` back to robot

```bash
ros2 topic echo /scan --once     # PE → DE
ros2 topic echo /cmd_vel --once  # DE → PE
```

### ② State Synchronisation
`twin_safety_node` publishes the **same command** simultaneously to:
- `/cmd_vel` → real robot (Physical Entity)
- `/sim/cmd_vel` → Gazebo twin (Digital Entity simulation)

`dt_logger_node` tracks both positions and computes `sync_error_m`:

```bash
ros2 topic echo /sim/cmd_vel     # same values as /cmd_vel → motion sync
ros2 topic echo /dt/status --full-length   # real/sim_position, sync_error_m, action counts
ros2 service call /get_twin_status std_srvs/srv/Trigger
ros2 service call /get_dt_status   std_srvs/srv/Trigger
# sync_error_m close to 0 → strong State Synchronisation evidence
```

> **Use `--full-length`** on `ros2 topic echo /dt/status`. Without it, ros2 cuts
> the JSON off with `...` after the first ~100 chars, so fields like
> `fertilize_actions` / `spray_actions` get hidden. Or just call
> `ros2 service call /get_dt_status std_srvs/srv/Trigger` for the full pretty JSON.

### ③ Object/Environment Interaction
**A. Safety stop:** LiDAR detects obstacle < 0.25 m → robot stops automatically.

**B. Farm zone actions:** Robot navigates to farm zones → DE logs spray/fertilize
action with timestamp.

```bash
ros2 topic echo /farm_action
ros2 service call /get_dt_log std_srvs/srv/Trigger
```

---

## System Architecture

```
Physical Entity (TurtleBot3 Burger)
  /scan ──────────► twin_safety_node ──► /cmd_vel     → real robot
  /odom ──────────► zone_monitor_node    /sim/cmd_vel → Gazebo twin
                         │
                   /farm_action
                         │
                    dt_logger_node ──► /dt/status (every 5s)

Gazebo twin (PushRosNamespace 'sim'):
  /sim/scan  → twin_safety_node
  /sim/odom  → zone_monitor_node, navigator_node
  /sim/cmd_vel ← twin_safety_node
```

---

## Nodes, Topics, Services

| Concept | Where |
|---|---|
| **Nodes** | `twin_safety_node`, `zone_monitor_node`, `dt_logger_node`, `navigator_node` (reactive), `nav2_navigator` (Nav2) |
| **Topics** | `/scan`, `/odom`, `/cmd_vel`, `/sim/cmd_vel`, `/farm_action`, `/dt/status`, `/navigator/status` |
| **Services** | `/get_twin_status`, `/get_zone_status`, `/get_dt_status`, `/get_dt_log`, `/start_navigation`, `/return_home`, `/stop_navigation`, `/nav_status` |

> **Two navigation modes:**
> - `navigator_node` (reactive, executable `navigator_node`, file `navigator.py`) — hand-coded go-to-goal + obstacle avoidance. No map needed. Used by `farm_twin.launch.py`.
> - `nav2_navigator` (Nav2 Simple Commander, executable `nav2_navigator`, file `navigator_node.py`) — Nav2 plans the path. Used by `navigation.launch.py` (lab — needs your real SLAM map `~/map.yaml`) and `gazebo_nav2_demo.launch.py` (home — uses the bundled `maps/lab_map.yaml`, no SLAM needed). Adds auto return-home on low battery.

---

## Package Structure

```
farm_twin_poc/
├── farm_twin_poc/
│   ├── twin_safety_node.py    Node 2 — safety stop + state sync
│   ├── zone_monitor_node.py   Node 3 — farm zone detection
│   ├── dt_logger_node.py      Node 4 — Digital Entity logger
│   ├── navigator.py           Node 5a — reactive navigation (no map needed)
│   └── navigator_node.py      Node 5b — Nav2 navigation + return-home (needs map)
├── launch/
│   ├── gazebo_twin.launch.py     Gazebo Digital Twin (lab_world.sdf)
│   ├── farm_twin.launch.py       Nodes 2+3+4+5a (reactive full system)
│   ├── slam.launch.py            SLAM with Cartographer
│   ├── gazebo_nav2_demo.launch.py  Nav2 autonomous demo at home (Gazebo)
│   └── navigation.launch.py      Nav2 + nodes 3+4+5b on real robot (lab)
├── scripts/
│   ├── map_to_world.py        SLAM map → Gazebo SDF world
│   └── world_to_map.py        Gazebo SDF world → Nav2 map (bundled at-home map)
├── worlds/
│   └── lab_world.sdf          Lab room (with flat colored zone tiles)
└── maps/
    └── lab_map.yaml / .pgm    Nav2 map generated from lab_world.sdf (bundled)
```

---

## Farm Zones

4 predefined zones marked as **flat colored floor tiles** (0.45 × 0.45 m) in Gazebo:
- 🔴 **Red** = spray zones (Striga herbicide)
- 🟢 **Green** = fertilize zones (NPK application)

> Tiles are deliberately flat (~2 cm tall) and visual-only. An earlier version
> used raised spheres, but in gz-sim the LiDAR raytraces against **visual**
> geometry, so a sphere reaching up to the LiDAR plane (~0.18 m) was seen as an
> obstacle — the robot stopped short and never reached the centre to trigger an
> action. A low tile sits under the LiDAR beam, so the robot drives over it and
> `zone_monitor_node` fires `/farm_action`. The tile is inscribed in the 0.35 m
> trigger circle, so the action logs the instant the robot touches the tile.
> When the robot reaches a zone it **spins 360° in place** to signal the
> spray/fertilize action (visible in Gazebo + RViz).

Coordinates are in the odom frame (metres from robot start position).
Update in **all three** places before lab session:
- `farm_twin_poc/zone_monitor_node.py` → `FARM_ZONES`
- `farm_twin_poc/navigator.py` → `WAYPOINTS`  (reactive mode)
- `farm_twin_poc/navigator_node.py` → `WAYPOINTS`  (Nav2 mode)

---

## The `source` Command

Every new terminal needs these lines before any `ros2` command:

**At home (Docker):**
```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

**At lab (Linux native):**
```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=<ROBOT_ID>   # last number of robot IP e.g. 192.168.8.40 → 40
```

---

---

# PART A — AT HOME (Windows + WSL + Docker)

---

## A1. Start Docker (every session)

1. Open **Docker Desktop** on Windows
2. Open **Ubuntu** from Start Menu (not PowerShell)
3. Start container:
```bash
docker run --rm -it --name turtlebot3_container --net=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/c2irr10/turtlebot3_ws:/ws \
  --user $(id -u):$(id -g) turtlebot3_ws bash
```

**Every additional terminal:**
```bash
docker exec -it turtlebot3_container bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

> `Error: No such container` → Docker not running. Repeat step 3.

---

## A2. Clone and build (first time only)

```bash
cd /ws/src
git clone https://github.com/giahymai/CBL_DigitalTwin.git farm_twin_poc

cd /ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build --packages-select farm_twin_poc
source install/setup.bash
```

Verify:
```bash
ros2 pkg executables farm_twin_poc
# Expected: dt_logger_node, nav2_navigator, navigator_node, safety_stop_node, twin_safety_node, zone_monitor_node
```

> **Build error "failed to create symbolic link":**
> ```bash
> cd /ws && rm -rf build/ install/ log/
> colcon build --packages-select farm_twin_poc && source install/setup.bash
> ```

---

## A3. Test simulation at home (2 modes)

### Mode 1 — Manual teleop

**Terminal 1 — Gazebo:**
```bash
ros2 launch farm_twin_poc gazebo_twin.launch.py
```

**Terminal 2 — Farm Twin nodes:**
```bash
ros2 launch farm_twin_poc farm_twin.launch.py
# Default: lab:=false (uses /sim/scan, /sim/odom from Gazebo)
```

**Terminal 3 — Teleop:**
```bash
ros2 run turtlebot3_teleop teleop_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

- Drive toward wall with `w` → robot stops automatically (safety stop)
- Drive onto a colored floor tile → farm action triggers, logged to `/farm_action`
- `ros2 topic echo /sim/cmd_vel` → same values as `/cmd_vel` (State Sync)

> **Auto-spin in teleop:** the 360° spray-spin is built into the *navigators*,
> so it fires in autonomous mode (Mode 2/3), not while driving by hand. To get
> the spin during manual teleop, run `zone_monitor_node` yourself with
> `spin_on_entry:=true` instead of letting `farm_twin.launch.py` start it (do
> NOT run both — they would both publish `/cmd_vel_raw`):
> ```bash
> ros2 run farm_twin_poc zone_monitor_node --ros-args \
>     -p odom_topic:=/sim/odom -p spin_on_entry:=true
> ```

### Mode 2 — Autonomous navigation

**Terminal 1 — Gazebo** (same as above)

**Terminal 2 — Farm Twin nodes** (same as above)

**Terminal 3 — Start autonomous run:**
```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```

Robot navigates to each zone autonomously, avoids obstacles, and spins 360° at
each zone to signal the spray/fertilize operation.

Monitor:
```bash
ros2 topic echo /navigator/status   # current zone, state
ros2 topic echo /farm_action        # zone actions triggering
ros2 service call /nav_status std_srvs/srv/Trigger
```

Stop:
```bash
ros2 service call /stop_navigation std_srvs/srv/Trigger
```

Return to start position (manual):
```bash
ros2 service call /return_home std_srvs/srv/Trigger
```

Return home automatically when battery is low (< 20% or < 11V) — no action needed, triggers automatically at lab with real robot.

### Mode 3 — Nav2 autonomous navigation (planned path)

Use this when you want **Nav2 to plan the path** to each zone instead of the
reactive go-to-goal. Runs in the lab world (no `sim` namespace) so the TF tree
is clean.

**No map needed at home** — the package ships `maps/lab_map.yaml`, generated
from `lab_world.sdf` via `scripts/world_to_map.py`, so the Gazebo walls and the
Nav2 map match exactly. The robot spawns at the world origin (0, 0) so that
`/odom` lines up with the map frame and the farm zones trigger correctly.

**Single terminal — everything (Gazebo + Nav2 + nodes):**
```bash
# Machine WITH a GPU (native Linux lab laptop, or PC with a graphics card):
ros2 launch farm_twin_poc gazebo_nav2_demo.launch.py

# Machine WITHOUT a GPU (e.g. Docker-in-WSL at home): run Gazebo headless
ros2 launch farm_twin_poc gazebo_nav2_demo.launch.py headless:=true

# Override the map only if you really want your own: map:=$HOME/map.yaml
```

> ⚠️ **No GPU? Use `headless:=true`.** Gazebo needs a GPU to render the 3D
> window AND the robot's LiDAR. On a GPU-less machine the CPU can't keep up, the
> sim clock stutters ("Detected jump back in time" floods the log), Nav2's
> controller chokes and the robot freezes. `headless:=true` drops the 3D window
> to cut that load — you still get **RViz** (map + robot + path) for the demo.
> If even headless stalls, the container simply has no GPU access; run the demo
> on a machine with a graphics card. **The lab session is unaffected** — there
> Nav2 drives the real robot in real time, with no Gazebo and no sim clock.

> Regenerate the bundled map only if you change the **walls** in
> `lab_world.sdf` (moving zone markers does not need it):
> ```bash
> python3 scripts/world_to_map.py worlds/lab_world.sdf maps/lab_map
> colcon build --packages-select farm_twin_poc && source install/setup.bash
> ```

Wait until the log shows **"Nav2 is active"**. By default this demo seeds the
initial pose automatically (`set_initial_pose:=true`); if it didn't, click
**"2D Pose Estimate"** in RViz at the robot spawn (0, 0). Then:
```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```

Monitor / control (same services as the other modes):
```bash
ros2 topic echo /navigator/status    # state, current zone, battery, completed
ros2 service call /nav_status      std_srvs/srv/Trigger
ros2 service call /return_home     std_srvs/srv/Trigger   # go home now
ros2 service call /stop_navigation std_srvs/srv/Trigger
```

> **Battery in Gazebo:** Gazebo TB3 usually does **not** publish
> `/battery_state`, so auto return-home will not fire on its own. To test the
> logic at home, fake a low battery while navigating:
> ```bash
> ros2 topic pub /battery_state sensor_msgs/msg/BatteryState "{percentage: 0.15}" --once
> ```
> The robot should cancel its goal and drive back to `home_x/home_y`.

---

## A4. Push to GitHub

```bash
cd /home/c2irr10/turtlebot3_ws/src/farm_twin_poc
find . -name "*Zone.Identifier*" -delete
git add .
git commit -m "Update"
git push
```

---

---

# PART B — LAB SESSION

**Linux native — no Docker, no WSL.**
**Workspace: `~/turtlebot3_ws`**
**ROS_DOMAIN_ID = last number of robot IP** (e.g. 192.168.8.40 → 40)

---

## B1. Setup (first time on lab laptop)

Connect to WiFi **"AP2IRR10"**, then:

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/giahymai/CBL_DigitalTwin.git farm_twin_poc

cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
colcon build --packages-select farm_twin_poc
source install/setup.bash
```

---

## B2. Connect to robot

**Terminal 1 — SSH:**
```bash
ssh turtlebot@<ROBOT_IP>
```
Inside robot:
```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
ros2 launch turtlebot3_bringup robot.launch.py
```

Verify from laptop:
```bash
export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 topic hz /scan    # must show ~10 Hz
```

---

## B3. Run SLAM to map the lab room

**Terminal 2 — Cartographer:**
```bash
source /opt/ros/jazzy/setup.bash && source /opt/turtlebot3_ws/install/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger && export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 launch farm_twin_poc slam.launch.py
```

**Terminal 3 — Drive robot around room:**
```bash
export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 run turtlebot3_teleop teleop_keyboard
```

**Terminal 4 — Save map:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 run nav2_map_server map_saver_cli -f ~/map \
    --ros-args -p map_topic:=/map
```

---

## B4. Run the full Digital Twin

Open **5 terminals** (each needs full source block + `ROS_DOMAIN_ID`).

**Terminal 1 — Robot bringup** (already running from B2).

**Terminal 2 — Gazebo twin:**
```bash
ros2 launch farm_twin_poc gazebo_twin.launch.py
```

**Terminal 3 — Farm Twin system:**
```bash
ros2 launch farm_twin_poc farm_twin.launch.py lab:=true
# lab:=true switches all topics to real robot: /scan, /odom
```

**Terminal 4 — Teleop (manual mode):**
```bash
ros2 run turtlebot3_teleop teleop_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

**Terminal 5 — Monitor:**
```bash
ros2 topic echo /sim/cmd_vel
```

---

## B5. Autonomous navigation at lab

After B4 is running:

```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```

Robot drives autonomously to each farm zone, avoids obstacles, and spins 360° at
each zone. Zone actions logged by Digital Entity automatically.

Monitor:
```bash
ros2 topic echo /navigator/status
ros2 topic echo /farm_action
ros2 service call /get_dt_log std_srvs/srv/Trigger
```

Stop anytime:
```bash
ros2 service call /stop_navigation std_srvs/srv/Trigger
```

---

## B5-bis. Nav2 autonomous navigation at lab (planned path)

Tutor-recommended mode: zones are fixed and you already have a SLAM map, so
let **Nav2 plan the path**. This replaces the reactive run above — do **not**
run `farm_twin.launch.py` autonomous at the same time (they both publish
motion). Robot bringup (B2) must be running, and you need `~/map.yaml` (B3).

**Terminal — Nav2 + navigator + zone monitor + DT logger (one command):**
```bash
ros2 launch farm_twin_poc navigation.launch.py \
    map:=~/map.yaml home_x:=0.0 home_y:=0.0
```
Replace `home_x/home_y` with the robot's REAL start coordinates in the lab room
(the spot you want it to return to). Leave `set_initial_pose:=false` (default)
on the real robot.

In RViz, click **"2D Pose Estimate"** at the robot's actual position, wait for
**"Nav2 is active"**, then:
```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```

The robot drives a Nav2-planned path to each zone; `zone_monitor` fires
`/farm_action` and `dt_logger` logs each spray/fertilize. Auto return-home on
low battery works for real here (real TB3 publishes `/battery_state`).

---

## B6. Battery monitoring + Return home

The robot automatically returns to its start position when battery is low
(< 20% or < 11V). This triggers without any manual intervention.

**Manual return home at any time:**
```bash
ros2 service call /return_home std_srvs/srv/Trigger
```

Monitor battery + navigation state:
```bash
ros2 topic echo /navigator/status
# Shows: state=... | battery=85% | returning=False | completed=[...]
```

The home position depends on which navigator you run:
- **Reactive** (`navigator_node`, via `farm_twin.launch.py`): home is recorded
  automatically when the robot first publishes `/odom` — no coordinates needed.
- **Nav2** (`nav2_navigator`, via `navigation.launch.py`): home is set by the
  `home_x` / `home_y` / `home_yaw` launch args. Set them to the robot's real
  start pose, or it will return to the wrong spot.

> **At home (Gazebo):** `/battery_state` may not publish → battery monitoring
> inactive. Use `/return_home` manually, or fake a low battery to test:
> ```bash
> ros2 topic pub /battery_state sensor_msgs/msg/BatteryState "{percentage: 0.15}" --once
> ```

---

## B7. Demo the 3 DTAS criteria

### ① Bi-directional Communication
```bash
ros2 topic echo /scan --once      # PE → DE
ros2 topic echo /cmd_vel --once   # DE → PE
```

### ② State Synchronisation
```bash
ros2 topic echo /sim/cmd_vel      # same values as /cmd_vel → motion sync
ros2 service call /get_twin_status std_srvs/srv/Trigger
```

### ③ Object/Environment Interaction
**Safety stop:** Place object in front → robot stops.

**Autonomous zone navigation:**
```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
ros2 topic echo /farm_action
ros2 service call /get_dt_log std_srvs/srv/Trigger
```

---

## B8. All monitoring commands

```bash
ros2 topic echo /farm_action
ros2 topic echo /dt/status --full-length   # --full-length or the counts get cut by "..."
ros2 topic echo /sim/cmd_vel
ros2 topic echo /navigator/status
ros2 node list
ros2 topic list

ros2 service call /get_twin_status  std_srvs/srv/Trigger
ros2 service call /get_zone_status  std_srvs/srv/Trigger
ros2 service call /get_dt_status    std_srvs/srv/Trigger
ros2 service call /get_dt_log       std_srvs/srv/Trigger
ros2 service call /start_navigation std_srvs/srv/Trigger
ros2 service call /stop_navigation  std_srvs/srv/Trigger
ros2 service call /return_home      std_srvs/srv/Trigger
ros2 service call /nav_status       std_srvs/srv/Trigger
```

---

## B9. Adjust zone positions

Drive robot to each zone location, check tọa độ:
```bash
ros2 topic echo /odom --once | grep -A3 "position"
```

Update same coordinates in all three places:
- `farm_twin_poc/zone_monitor_node.py` → `FARM_ZONES`
- `farm_twin_poc/navigator.py` → `WAYPOINTS`  (reactive)
- `farm_twin_poc/navigator_node.py` → `WAYPOINTS`  (Nav2)

Also update `<pose>` in `worlds/lab_world.sdf` for visual markers.

```bash
git add . && git commit -m "Update zone coordinates" && git push
git pull && colcon build --packages-select farm_twin_poc && source install/setup.bash
```

---

## B10. Shutdown procedure

1. `Ctrl+C` — Terminal 4 (teleop)
2. `Ctrl+C` — Terminal 3 (farm twin nodes)
3. `Ctrl+C` — Terminal 2 (Gazebo)
4. Terminal 1: `sudo shutdown now` — wait for SSH to drop
5. Flip power switch on robot

---

## Troubleshooting

**`ros2` not found:** `source /opt/ros/jazzy/setup.bash`

**Package not found:** `source install/setup.bash` (or `/ws/install/setup.bash` in Docker)

**`turtlebot3_description` not found:** `source /opt/turtlebot3_ws/install/setup.bash`

**No `/scan` from robot:** Check bringup (T1) and `ROS_DOMAIN_ID`.

**Safety stop not working:** `ros2 topic hz /scan` — must show ~10 Hz.

**x=y=0 in `/dt/status`:** Wrong `odom_topic` — check `farm_twin.launch.py`.

**Zone actions not triggering:** Drive within `radius` metres (default 0.35 m).
Check `ros2 topic hz /odom`.

**Zone actions not triggering under Nav2 (robot is visibly on the tile but
`spray_actions` stays 0):** frame mismatch. `FARM_ZONES` and the tiles live in
the **map** frame, but under Nav2 `/odom` drifts (AMCL corrects it via a
`map->odom` transform), so the robot reaches the tile in map coords while
`/odom` reads a different number. The Nav2 launches fix this by running
`zone_monitor_node` with `position_source:=tf` (looks up `map->base_link`).
Reactive/teleop keep `position_source:=odom` (no map, `/odom` ≈ world).

**Robot stuck during navigation:** Automatic escape after 4s — wait or call `/stop_navigation`.

**Robot not returning home:** Check `/odom` is publishing. Call `/nav_status` to see home position.

**"failed to spin map subscription":**
```bash
export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p map_topic:=/map
```

**Build error "failed to create symbolic link":**
```bash
cd ~/turtlebot3_ws && rm -rf build/ install/ log/
colcon build --packages-select farm_twin_poc && source install/setup.bash
```
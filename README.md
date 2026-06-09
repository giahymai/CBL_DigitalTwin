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

## What Does This PoC Prove? (the three assignment goals)

The PoC is built to demonstrate **exactly three things**, each with a concrete,
reproducible piece of evidence. Everything else in the package exists only to
serve these three goals.

### ① Bi-directional Communication
Data flows **both ways** between the Physical Entity (robot) and the Digital
Entity (the ROS 2 nodes):

- **PE → DE**: the robot streams `/scan` (LiDAR) and `/odom` (pose) up to the nodes.
- **DE → PE**: the nodes send motion commands back down on `/cmd_vel`.

**Evidence:**
```bash
ros2 topic echo /scan    --once    # PE → DE : sensor data arriving from the robot
ros2 topic echo /odom    --once    # PE → DE : pose arriving from the robot
ros2 topic echo /cmd_vel --once    # DE → PE : command going back to the robot
```
Seeing live data on `/scan` + `/odom` **and** on `/cmd_vel` at the same time is
the proof: the link is not one-way.

### ② State Synchronisation
The Digital Entity keeps a **simulated twin** of the robot in lock-step with the
physical one. `twin_safety_node` publishes the **identical command** to two
places at the same instant:

- `/cmd_vel` → the real robot (Physical Entity)
- `/sim/cmd_vel` → the Gazebo twin (Digital Entity simulation)

`dt_logger_node` then watches both poses and computes `sync_error_m`
(the distance between real pose and twin pose).

**Evidence:**
```bash
# Same numbers appear on both topics → the twin mirrors the robot's motion
ros2 topic echo /cmd_vel
ros2 topic echo /sim/cmd_vel

# sync_error_m close to 0 → real pose and twin pose stay aligned
ros2 topic echo /dt/status --full-length
ros2 service call /get_twin_status std_srvs/srv/Trigger
ros2 service call /get_dt_status   std_srvs/srv/Trigger
```

> **Use `--full-length`** on `ros2 topic echo /dt/status`. Without it, ros2 cuts
> the JSON off with `...` after the first ~100 chars, so fields like
> `fertilize_actions` / `spray_actions` get hidden. The two services below give
> the same data without the truncation:
> ```bash
> ros2 service call /get_dt_status std_srvs/srv/Trigger   # snapshot: counts, sync_error_m, last_action
> ros2 service call /get_dt_log    std_srvs/srv/Trigger   # FULL history: every spray/fertilize event + timestamp
> ```

### ③ Obstacle Avoidance & Object/Environment Interaction
The robot perceives its surroundings and **acts on them**, in two ways:

**A. Reflex safety stop (twin_safety_node).** A close obstacle on the LiDAR
(< 0.25 m in the front arc) overrides the command and stops forward motion
(turning is still allowed). This is the fast, low-level reflex.

**B. Planned obstacle avoidance (Nav2).** During autonomous navigation, **Nav2
is the path planner**: it builds a costmap from `/scan`, plans a global path to
each farm zone, and steers around walls/obstacles locally. The robot never
gropes blindly — it follows a planned, collision-free path.

**C. Environment interaction (farm zones).** When the robot reaches a farm zone,
`zone_monitor_node` fires a `/farm_action` (spray/fertilize) and the robot spins
360° in place to signal the operation; `dt_logger_node` logs it with a timestamp.

**Evidence:**
```bash
# A — safety stop: drive toward a wall in teleop, the robot stops short
ros2 service call /get_twin_status std_srvs/srv/Trigger   # shows real_blocked / times_blocked

# B + C — Nav2 plans a path to each zone, avoids obstacles, logs the action
ros2 topic echo /farm_action
ros2 service call /get_dt_log std_srvs/srv/Trigger
```

---

## System Architecture

```
Physical Entity (TurtleBot3 Burger)
  /scan ──────────► twin_safety_node ──► /cmd_vel     → real robot     (Goal ① + ③A)
  /odom ──────────► zone_monitor_node    /sim/cmd_vel → Gazebo twin    (Goal ②)
                         │
                   /farm_action
                         │
                    dt_logger_node ──► /dt/status (every 5 s)          (Goal ②)

Gazebo twin (PushRosNamespace 'sim'):
  /sim/scan  → twin_safety_node
  /sim/odom  → zone_monitor_node
  /sim/cmd_vel ← twin_safety_node

Autonomous navigation (Goal ③B):
  Nav2 (nav2_navigator) ──► plans path ──► /cmd_vel ──► robot, avoiding obstacles
  zone_monitor_node ──► /farm_action ──► dt_logger_node
```

The PoC has two run-time stacks that demonstrate different goals:

- **Digital-Twin stack** (`gazebo_twin.launch.py` + `farm_twin.launch.py`,
  driven by teleop) → Goals ① and ② (and the reflex safety stop, ③A).
- **Nav2 autonomous stack** (`gazebo_nav2_demo.launch.py` at home /
  `navigation.launch.py` at the lab) → Goal ③ (planned obstacle avoidance +
  zone actions).

> They are **separate runs**: `twin_safety_node` filters teleop into `/cmd_vel`,
> while Nav2 drives `/cmd_vel` directly. Running both at once would make two
> sources fight over the robot, so do **one at a time**.

---

## Nodes, Topics, Services

| Concept | Where |
|---|---|
| **Nodes** | `twin_safety_node`, `zone_monitor_node`, `dt_logger_node`, `nav2_navigator` (Nav2 path planner), `safety_stop_node` (standalone sim safety stop) |
| **Topics** | `/scan`, `/odom`, `/cmd_vel`, `/sim/cmd_vel`, `/farm_action`, `/dt/status`, `/navigator/status` |
| **Services** | `/get_twin_status`, `/get_zone_status`, `/get_dt_status`, `/get_dt_log`, `/start_navigation`, `/return_home`, `/stop_navigation`, `/nav_status` |

> **One navigator only:** `nav2_navigator` (executable `nav2_navigator`, file
> `navigator_node.py`) uses the **Nav2 Simple Commander** — Nav2 plans the path
> to each zone and handles obstacle avoidance. Used by `navigation.launch.py`
> (lab — needs your real SLAM map `~/map.yaml`) and `gazebo_nav2_demo.launch.py`
> (home — uses the bundled `maps/lab_map.yaml`, no SLAM needed). It also does
> auto return-home on low battery.

---

## Package Structure

```
farm_twin_poc/
├── farm_twin_poc/
│   ├── twin_safety_node.py    Node 2 — safety stop + state sync (Goals ① ② ③A)
│   ├── zone_monitor_node.py   Node 3 — farm zone detection (Goal ③C)
│   ├── dt_logger_node.py      Node 4 — Digital Entity logger (Goal ②)
│   ├── navigator_node.py      Node 5 — Nav2 navigation + return-home (Goal ③B)
│   └── safety_stop_node.py    standalone sim-only safety stop (optional)
├── launch/
│   ├── gazebo_twin.launch.py        Gazebo Digital Twin (lab_world.sdf)
│   ├── farm_twin.launch.py          Nodes 2+3+4 — twin/state-sync stack (teleop)
│   ├── slam.launch.py               SLAM with Cartographer (build the lab map)
│   ├── gazebo_nav2_demo.launch.py   Nav2 autonomous demo at home (Gazebo)
│   └── navigation.launch.py         Nav2 + nodes 3+4 on the real robot (lab)
├── scripts/
│   ├── map_to_world.py        SLAM map → Gazebo SDF world
│   └── world_to_map.py        Gazebo SDF world → Nav2 map (bundled at-home map)
├── worlds/
│   └── lab_world.sdf          Lab room (with flat colored zone tiles)
├── maps/
│   └── lab_map.yaml / .pgm    Nav2 map generated from lab_world.sdf (bundled)
└── config/
    └── nav2_sim.yaml          Nav2 params tuned for the slow (GPU-less) sim
```

---

## Farm Zones

4 predefined zones marked as **flat colored discs** (radius 0.35 m) in Gazebo.
- **Red** = spray zones (Striga herbicide) — visited first
- **Green** = fertilize zones (NPK application) — visited second

Layout (Gazebo top-down view — robot spawns top-left at `(3, 3)`):
```
TOP-LEFT  spray_zone_C   (3.5, 2.7) [red]    spray_zone_A   (0.5, 2.7) [red]    TOP-RIGHT
BOTTOM-LEFT fertilize_zone_B (3.5, 0.7) [green]  fertilize_zone_D (0.5, 0.7) [green]  BOTTOM-RIGHT
                                                                             ★ (0,0)
```
Tour order: C (top-left, near spawn) → A (top-right) → D (bottom-right) → B (bottom-left).

> Discs are deliberately flat (~2 cm tall) and visual-only. An earlier version
> used raised spheres, but in gz-sim the LiDAR raytraces against **visual**
> geometry, so a sphere reaching up to the LiDAR plane (~0.18 m) was seen as an
> obstacle — the robot stopped short and never reached the centre to trigger an
> action. A low disc sits under the LiDAR beam, so the robot drives over it and
> `zone_monitor_node` fires `/farm_action`. The disc radius equals the 0.35 m
> trigger radius, so the visible zone and the detected zone line up exactly.
> When the robot reaches a zone it **spins 360° in place** to signal the
> spray/fertilize action (visible in Gazebo + RViz).

Coordinates are in the map/odom frame (metres from robot start position).
Update in **both** places before the lab session:
- `farm_twin_poc/zone_monitor_node.py` → `FARM_ZONES`
- `farm_twin_poc/navigator_node.py` → `WAYPOINTS`  (Nav2 goals)

…and the `<pose>` of each zone disc in `worlds/lab_world.sdf` for the visuals.

---

## Why the Map Looks Different at Lab vs At Home

This is a common point of confusion. There are two completely different map sources:

**`maps/lab_map.yaml`** (bundled in the repo):
- A **real SLAM scan** of the lab room made with Cartographer
- Used both at home (loaded into Gazebo Nav2 demo) and at lab (loaded by AMCL)
- `worlds/lab_world.sdf` was generated FROM this map via `scripts/map_to_world.py`,
  so the Gazebo walls and the Nav2 map match each other exactly

**At home (Gazebo):**
- Robot spawns at world `(0, 0)` — the point in the SDF that corresponds to where
  the robot was standing when the original SLAM scan was made
- Visually this appears at a specific corner of the Gazebo room

**At lab (real robot):**
- The map frame `(0, 0)` is the same physical spot — where SLAM was originally run
- If the robot is placed at a **different physical location**, AMCL will show it at a
  different position on the map (e.g. top-left instead of bottom-right)
- This is normal — use "2D Pose Estimate" in RViz2 to tell AMCL where the robot is,
  then read `/amcl_pose` to get the exact map-frame coordinates for `home_x/home_y`

**Consequence for zone coordinates:** `WAYPOINTS` and `FARM_ZONES` are in the map
frame. They must be measured by driving the robot to each zone and reading `/amcl_pose`
(see B9). Once measured and committed, they stay valid as long as zones don't move.

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
# Expected: dt_logger_node, nav2_navigator, safety_stop_node, twin_safety_node, zone_monitor_node
```

> **Build error "failed to create symbolic link":**
> ```bash
> cd /ws && rm -rf build/ install/ log/
> colcon build --packages-select farm_twin_poc && source install/setup.bash
> ```

---

## A3. Test simulation at home (2 modes)

### Mode 1 — Digital-Twin demo (teleop) → proves Goals ① ② ③A

This stack shows **bi-directional communication**, **state synchronisation**,
and the **reflex safety stop**. You drive by hand; `twin_safety_node` mirrors
every command into the Gazebo twin and blocks forward motion near obstacles.

**Terminal 1 — Gazebo twin:**
```bash
ros2 launch farm_twin_poc gazebo_twin.launch.py
```

**Terminal 2 — Twin / state-sync nodes:**
```bash
ros2 launch farm_twin_poc farm_twin.launch.py
# Default: lab:=false (uses /sim/scan, /sim/odom from Gazebo)
```

**Terminal 3 — Teleop:**
```bash
ros2 run turtlebot3_teleop teleop_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

- Drive toward a wall with `w` → robot stops automatically (**Goal ③A** safety stop)
- `ros2 topic echo /sim/cmd_vel` → same values as `/cmd_vel` (**Goal ②** state sync)
- `ros2 topic echo /scan --once` + `/cmd_vel --once` (**Goal ①** bi-directional)
- Drive onto a colored floor tile → `/farm_action` fires (**Goal ③C** zone action)

> **Auto-spin in teleop:** the 360° spray-spin is built into the *Nav2 navigator*,
> so it fires in autonomous mode (Mode 2), not while driving by hand. To get the
> spin during manual teleop, run `zone_monitor_node` yourself with
> `spin_on_entry:=true` instead of letting `farm_twin.launch.py` start it (do
> NOT run both — they would both command motion):
> ```bash
> ros2 run farm_twin_poc zone_monitor_node --ros-args \
>     -p odom_topic:=/sim/odom -p spin_on_entry:=true
> ```

### Mode 2 — Nav2 autonomous navigation (planned path) → proves Goal ③B

This is the **only autonomous mode**: **Nav2 plans the path** to each zone and
avoids obstacles. It runs in the lab world WITHOUT the `sim` namespace so the TF
tree is clean (`map -> odom -> base_link`) and Nav2 works.

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

> **No GPU? Use `headless:=true`.** Gazebo needs a GPU to render the 3D
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

Robot drives a **Nav2-planned path** to each zone, avoids obstacles, and spins
360° at each zone to signal the spray/fertilize operation.

Monitor / control:
```bash
ros2 topic echo /navigator/status    # state, current zone, battery, completed
ros2 topic echo /farm_action         # zone actions triggering (Goal ③C)
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

**Linux native — no Docker, no WSL. Workspace: `~/turtlebot3_ws`.**

## LAB SETUP BLOCK — run this FIRST in EVERY new laptop terminal

Every new terminal **on the laptop** is empty: it knows no `ros2`, no package,
no robot. Paste this block **before any other command**. Nothing in B3–B8 works
without it.

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=<ROBOT_ID>     # last number of the robot's IP, e.g. 192.168.8.40 → 40
```

> - `<ROBOT_ID>` must be the **same in every terminal** and match the robot.
> - Quick check it worked: `ros2 topic list` shows `/scan`. If not → wrong/missing
>   `ROS_DOMAIN_ID`, or robot bringup (B2) isn't running.
> - **The SSH terminal (B2) is the exception** — it runs *on the robot*, not the
>   laptop, so it uses the robot's own setup shown there, not this block.

Below, each laptop terminal says **"SETUP BLOCK, then:"** — that means run the
block above first, then the command shown.

---

## B0. Will it work at the lab? Pre-lab checklist

The real robot runs in real time (no Gazebo, no sim-clock stutter), and
`navigation.launch.py` uses the **stock** Nav2 params, so the at-home sim tuning
does not affect the lab. The logic is the same code. But these MUST be done or
the run will fail:

- [ ] **Built & sourced** on the lab laptop (B1), `ros2 pkg executables farm_twin_poc` lists all nodes.
- [ ] **Robot bringup running** (B2) and `ros2 topic hz /scan` ≈ 10 Hz from the laptop.
- [ ] **Map available**: the bundled `maps/lab_map.yaml` is a real SLAM scan of
      the lab room — copy it to `~/map.yaml` before launching (B3). If the room
      layout has changed since that scan, redo SLAM to get a fresh map.
- [ ] **Zone coordinates measured and updated** in `FARM_ZONES`, `WAYPOINTS`, and
      `lab_world.sdf` (B9), and `home_x/home_y` set to the robot's real start pose.
      Do this once per lab setup; if zones don't move between sessions, reuse the values.
- [ ] **AMCL localized**: in RViz, "2D Pose Estimate" at the robot's true spot
      (see B5 for how to do this).
- [ ] **LaserScan aligned with map**: in RViz, enable the LaserScan display and
      verify the red/yellow LiDAR lines land on the map walls. If they are offset,
      AMCL is poorly localized → robot will move erratically. Redo "2D Pose Estimate".
- [ ] **Test the spin early**: the 360° spin publishes straight to `/cmd_vel`. The
      lab's stock Nav2 runs `collision_monitor`, which may dampen it. If the spin
      is weak, raise `spin_speed` (param) or tell us to route the spin differently.

> Same `ROS_DOMAIN_ID` in every laptop terminal, matching the robot's IP.

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

**Terminal 1 — SSH into the robot** (this terminal runs ON the robot, so it uses
the robot's own setup, NOT the laptop SETUP BLOCK):
```bash
ssh turtlebot@<ROBOT_IP>
# then, inside the robot:
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02
ros2 launch turtlebot3_bringup robot.launch.py
```

**Terminal 2 — Verify from the laptop** — SETUP BLOCK, then:
```bash
ros2 topic hz /scan    # must show ~10 Hz → robot + ROS_DOMAIN_ID are correct
```

> **`/scan` not showing / 0 Hz:** this is the most common reason `navigation.launch.py`
> hangs with timeouts and RViz2 shows nothing. Nav2 waits for `/scan` on startup —
> if it never arrives, all Nav2 nodes timeout and abort before RViz2 even opens.
> Fix: make sure robot bringup (above) is running AND `ROS_DOMAIN_ID` is set
> identically in every laptop terminal.

---

## B3. Prepare the map

The bundled `maps/lab_map.yaml` is a **real SLAM scan of the lab room** — it is
ready to use directly. Copy it to the home directory so `navigation.launch.py`
finds it at the default path:

```bash
cp ~/turtlebot3_ws/install/farm_twin_poc/share/farm_twin_poc/maps/lab_map.yaml ~/map.yaml
cp ~/turtlebot3_ws/install/farm_twin_poc/share/farm_twin_poc/maps/lab_map.pgm ~/map.pgm
```

> **If the room layout has changed** (walls moved, large furniture added/removed)
> since the bundled map was made, redo SLAM to get a fresh map:
>
> **Terminal 2 — Cartographer** — SETUP BLOCK, then:
> ```bash
> ros2 launch farm_twin_poc slam.launch.py
> ```
> **Terminal 3 — Drive robot around entire room** — SETUP BLOCK, then:
> ```bash
> ros2 run turtlebot3_teleop teleop_keyboard
> ```
> Drive along all walls so the LiDAR sees everything.
>
> **Terminal 4 — Save map** — SETUP BLOCK, then:
> ```bash
> ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p map_topic:=/map
> ```
> Then commit the new map to the repo so it becomes the new bundled map:
> ```bash
> cp ~/map.yaml ~/turtlebot3_ws/src/farm_twin_poc/maps/lab_map.yaml
> cp ~/map.pgm  ~/turtlebot3_ws/src/farm_twin_poc/maps/lab_map.pgm
> cd ~/turtlebot3_ws/src/farm_twin_poc
> git add maps/ && git commit -m "Update SLAM map" && git push
> colcon build --packages-select farm_twin_poc && source install/setup.bash
> ```

---

## B4. Digital-Twin demo (teleop) → proves Goals ① ② ③A

Open **5 terminals**. Terminal 1 is the SSH/robot one from B2; terminals 2–5 are
laptop terminals → **SETUP BLOCK first in each**.

**Terminal 1 — Robot bringup** (already running from B2, on the robot).

**Terminal 2 — Gazebo twin** — SETUP BLOCK, then:
```bash
ros2 launch farm_twin_poc gazebo_twin.launch.py
```

> **Do NOT pass `x_pose` / `y_pose` here.** The default spawn `(3, 3)` is correct
> at both home and lab — the Gazebo world was built from the same SLAM map, so
> `(3, 3)` in Gazebo equals `(3, 3)` in the map frame at the lab too.
> Passing `x_pose:=0 y_pose:=0` puts the twin at the wrong corner of the room.

**Terminal 3 — Twin / state-sync system** — SETUP BLOCK, then:
```bash
ros2 launch farm_twin_poc farm_twin.launch.py lab:=true
# lab:=true switches all topics to the real robot: /scan, /odom
```

**Terminal 4 — Teleop** — SETUP BLOCK, then:
```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/cmd_vel_raw
```

**Terminal 5 — Monitor the sync** — SETUP BLOCK, then:
```bash
ros2 topic echo /sim/cmd_vel        # same as /cmd_vel → state synchronisation
```

Drive the robot: it stops at obstacles (③A), the twin mirrors its motion (②),
and sensor data flows up while commands flow down (①).

---

## B5. Nav2 autonomous navigation at lab (planned path) → proves Goal ③B

This is a **separate run** from B4. Robot bringup (B2) must be running and you
need `~/map.yaml` (B3) and updated zone coordinates (B9).

> **Do NOT run `farm_twin.launch.py` at the same time as this.** Both stacks
> publish to `/cmd_vel` and will fight each other — the robot will behave erratically.

**Terminal — Nav2 + navigator + zone monitor + DT logger** — SETUP BLOCK, then:
```bash
ros2 launch farm_twin_poc navigation.launch.py \
    map:=~/map.yaml home_x:=0.0 home_y:=0.0
```

Replace `home_x/home_y` with the robot's real start coordinates (see B9 — how to
read these values after AMCL localizes).

> **Nav2 params:** `navigation.launch.py` now uses `config/nav2_lab.yaml` by default.
> This file is tuned for the real robot: AMCL global recovery (re-localizes after
> dynamic obstacles scatter the particle cloud), `transform_tolerance: 1.0` (buffers
> WiFi latency on the `base_scan` TF), and `collision_monitor source_timeout: 3.0`
> (tolerates brief scan drops without stopping the robot). These reduce mid-navigation
> aborts compared to the stock turtlebot3 params. Override with `params_file:=...`
> if you need a different config.

RViz2 will open automatically once Nav2 finishes starting. **If RViz2 does not
appear**, it means Nav2 timed out — almost always because `/scan` is not arriving
(check B2) or `~/map.yaml` does not exist (check B3).

### Setting the initial pose in RViz2 (required)

AMCL needs to know where the robot is before it can localize. After RViz2 opens:

1. Look at the **toolbar** at the top of RViz2
2. Click **"2D Pose Estimate"**
3. On the map, **click on the robot's actual position** in the room
4. **Hold and drag** in the direction the robot is facing, then release

You will see a cluster of green arrows (AMCL particles) appear around that point.
Wait a few seconds for the particles to converge. Then verify localization is
correct: enable the **LaserScan** display in the left panel and check that the
red/yellow LiDAR lines align with the map walls. If they are offset, redo the
pose estimate more precisely.

Wait for the log to show **"Nav2 is active"**, then — in a new laptop terminal
(SETUP BLOCK first):
```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```

### What "feedback aborted" means

If `/start_navigation` returns `aborted`, or the robot aborts **mid-navigation**,
the most likely causes are:

1. **Zone coordinates not updated** — `WAYPOINTS` still has Gazebo sim coords,
   Nav2 tries to reach a point that is inside a wall or outside the map. Fix: do B9.
2. **AMCL not localized** — "2D Pose Estimate" not clicked, or clicked at the
   wrong position. Redo the pose estimate.
3. **Nav2 not active yet** — call `/nav_status` to check the state.
4. **Mid-navigation abort** (lab) — WiFi scan drop or dynamic obstacle scattered
   the AMCL particle cloud → `map→odom` TF drifted → DWB controller could not find
   a valid path → Nav2 BT aborted. `nav2_lab.yaml` reduces this with AMCL global
   recovery and relaxed timeouts, but it can still happen. Fix: redo "2D Pose
   Estimate" in RViz2, then call `/start_navigation` again.

### Why the robot moves slowly or does not follow the planned path

Nav2 shows a **global path** (the planned route) and a **local path** (what the
robot is actually doing right now). The local planner (DWB) reacts to real-time
sensor data, so the robot will not drive exactly along the global path — it
constantly adjusts to avoid obstacles and correct for localization drift.

If the robot moves very erratically or takes a completely wrong route:
- Open LaserScan in RViz2 and check alignment with map walls
- If walls are offset → AMCL is drifting → redo "2D Pose Estimate"
- There is **no conflict** between Nav2 and other nodes in this stack —
  `navigation.launch.py` does not run `twin_safety_node`

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

## B5b. Viewing Gazebo twin alongside Nav2 (optional, for demo)

To show **both** Gazebo (Digital Twin visual) and RViz2 (Nav2 map) at the same
time during the lab session — like the at-home demo — run these in addition to
`navigation.launch.py`:

**Extra Terminal A — Gazebo twin world** — SETUP BLOCK, then:
```bash
ros2 launch farm_twin_poc gazebo_twin.launch.py
```

**Extra Terminal B — Mirror real robot commands into Gazebo twin** — SETUP BLOCK, then:
```bash
ros2 run topic_tools relay /cmd_vel /sim/cmd_vel
```

`relay` copies every `/cmd_vel` message (published by Nav2 to drive the real
robot) onto `/sim/cmd_vel` in real time, so the Gazebo robot moves in sync with
the physical robot. This is the state synchronisation (Goal ②) made visible.

> Do **not** run `farm_twin.launch.py` alongside this — `twin_safety_node` also
> publishes to `/cmd_vel` and would conflict with Nav2.

---

## B6. Battery monitoring + Return home

During Nav2 navigation the robot automatically returns to its start position
when battery is low (< 20%). This triggers without any manual intervention.

**Manual return home at any time:**
```bash
ros2 service call /return_home std_srvs/srv/Trigger
```

Monitor battery + navigation state:
```bash
ros2 topic echo /navigator/status
# Shows: state=... | current=... | battery=85% | completed=[...]
```

The home position is set by the `home_x` / `home_y` / `home_yaw` launch args of
`navigation.launch.py` (lab) / `gazebo_nav2_demo.launch.py` (home). Set them to
the robot's real start pose, or it will return to the wrong spot.

> **At home (Gazebo):** `/battery_state` may not publish → battery monitoring
> inactive. Use `/return_home` manually, or fake a low battery to test:
> ```bash
> ros2 topic pub /battery_state sensor_msgs/msg/BatteryState "{percentage: 0.15}" --once
> ```

---

## B7. Demo the 3 assignment goals

### ① Bi-directional Communication
```bash
ros2 topic echo /scan    --once   # PE → DE
ros2 topic echo /odom    --once   # PE → DE
ros2 topic echo /cmd_vel --once   # DE → PE
```

### ② State Synchronisation  (run the B4 twin stack)
```bash
ros2 topic echo /sim/cmd_vel      # same values as /cmd_vel → motion sync
ros2 service call /get_twin_status std_srvs/srv/Trigger
ros2 service call /get_dt_status   std_srvs/srv/Trigger   # sync_error_m
```

### ③ Obstacle Avoidance & Object/Environment Interaction
**Reflex safety stop (③A):** place an object in front in teleop → robot stops.

**Planned avoidance + zone actions (③B + ③C):** run the B5 Nav2 stack:
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

## B9. Measure and update zone positions

Zone coordinates must be in the **map frame** (not the odom frame). After AMCL
has localized (LaserScan aligned with map walls), use `/amcl_pose` — not `/odom`
— to read positions.

Do this **once** when setting up the lab. If zone markers don't move between
sessions, the values stay valid.

### Step 1 — Read home position

Place the robot at its designated start position. After clicking "2D Pose
Estimate" in RViz2 and verifying LaserScan alignment:

```bash
ros2 topic echo /amcl_pose --once | grep -A5 "position"
```

Note the `x` and `y` — these are `home_x` and `home_y` for the launch command.

### Step 2 — Read each zone position

Physically place or mark each zone on the floor (tape works). Drive the robot
to the **centre** of each zone marker using teleop:

```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

Then read the position at each zone:

```bash
ros2 topic echo /amcl_pose --once | grep -A5 "position"
```

Record `x` and `y` for each of the 4 zones.

### Step 3 — Update the code (3 places, same x/y values)

**`farm_twin_poc/zone_monitor_node.py`** → `FARM_ZONES`:
```python
FARM_ZONES = [
    {'name': 'spray_1',     'type': 'spray',     'x': <x1>, 'y': <y1>, 'radius': 0.35},
    {'name': 'spray_2',     'type': 'spray',     'x': <x2>, 'y': <y2>, 'radius': 0.35},
    {'name': 'fertilize_1', 'type': 'fertilize', 'x': <x3>, 'y': <y3>, 'radius': 0.35},
    {'name': 'fertilize_2', 'type': 'fertilize', 'x': <x4>, 'y': <y4>, 'radius': 0.35},
]
```

**`farm_twin_poc/navigator_node.py`** → `WAYPOINTS`:
```python
WAYPOINTS = [
    (<x1>, <y1>),   # spray_1
    (<x2>, <y2>),   # spray_2
    (<x3>, <y3>),   # fertilize_1
    (<x4>, <y4>),   # fertilize_2
]
```

**`worlds/lab_world.sdf`** → `<pose>` of each zone model (for Gazebo visual):
```xml
<model name="spray_zone_A">
  <pose><x1> <y1> 0.015 0 0 0</pose>   <!-- only change x and y, keep 0.015 -->

<model name="spray_zone_C">
  <pose><x2> <y2> 0.015 0 0 0</pose>

<model name="fertilize_zone_B">
  <pose><x3> <y3> 0.015 0 0 0</pose>

<model name="fertilize_zone_D">
  <pose><x4> <y4> 0.015 0 0 0</pose>
```

### Step 4 — Rebuild and push

```bash
cd ~/turtlebot3_ws/src/farm_twin_poc
git add farm_twin_poc/zone_monitor_node.py \
        farm_twin_poc/navigator_node.py \
        worlds/lab_world.sdf
git commit -m "Update zone and home coordinates for real lab room"
git push

cd ~/turtlebot3_ws
colcon build --packages-select farm_twin_poc && source install/setup.bash
```

### Step 5 — Launch navigation with correct home position

```bash
ros2 launch farm_twin_poc navigation.launch.py \
    map:=~/map.yaml home_x:=<home_x> home_y:=<home_y>
```

---

## B10. Shutdown procedure

1. `Ctrl+C` — Terminal 4 (teleop)
2. `Ctrl+C` — Terminal 3 (farm twin nodes) / the Nav2 launch
3. `Ctrl+C` — Terminal 2 (Gazebo)
4. Terminal 1: `sudo shutdown now` — wait for SSH to drop
5. Flip power switch on robot

---

## Troubleshooting

**`ros2` not found:** `source /opt/ros/jazzy/setup.bash`

**Package not found:** `source install/setup.bash` (or `/ws/install/setup.bash` in Docker)

**`turtlebot3_description` not found:** `source /opt/turtlebot3_ws/install/setup.bash`

**No `/scan` from robot:** Check bringup (B2) and `ROS_DOMAIN_ID`. This is the
most common cause of Nav2 timeout and RViz2 not opening.

**RViz2 does not open / Nav2 keeps printing timeouts:**
`/scan` is not arriving. Verify `ros2 topic hz /scan` shows ~10 Hz from the laptop
terminal. If not → robot bringup not running or wrong `ROS_DOMAIN_ID`.

**Map is blank in RViz2 / "frame 'map' does not exist":**
AMCL has not published the `map → odom` transform yet. Click **"2D Pose Estimate"**
in RViz2 at the robot's actual position. Everything waiting on the `map` frame
(costmap, Nav2 controller) will unblock immediately after.

**Robot moves erratically / does not follow the Nav2 path:**
AMCL localization is drifting. Enable **LaserScan** in RViz2 and check that the
LiDAR lines align with the map walls. If offset, redo "2D Pose Estimate" more
carefully. This is not a node conflict — `navigation.launch.py` does not run
`twin_safety_node`.

**Robot aborts mid-navigation and disappears from RViz2 (lab):**
Chain reaction: WiFi scan drop → local costmap clears → DWB controller fails →
Nav2 BT aborts goal → AMCL stops receiving scan → `map→odom` TF expires → robot
disappears. `nav2_lab.yaml` (used by default) reduces this via AMCL global recovery,
`transform_tolerance: 1.0`, and `source_timeout: 3.0`. If it still occurs: redo
"2D Pose Estimate" in RViz2, wait for LaserScan to realign with walls, then call
`/start_navigation` again. Do **not** restart the whole launch — Nav2 is still running.

**`/start_navigation` returns "aborted":**
- Zone coordinates (`WAYPOINTS`) still have Gazebo sim values → do B9.
- Or AMCL not localized → redo "2D Pose Estimate".
- Or Nav2 not active yet → check `ros2 service call /nav_status std_srvs/srv/Trigger`.

**"Message Filter dropping message / queue is full" in RViz2:**
Visual-only warning. RViz2 cannot render all incoming messages fast enough over
WiFi. The robot and Nav2 are unaffected. To reduce the warning, disable unused
RViz2 displays (e.g. AMCL particle cloud) or set their `Queue Size` to `1`.

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
The teleop twin demo keeps `position_source:=odom` (no map, `/odom` ≈ world).

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

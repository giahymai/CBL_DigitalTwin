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

```bash
ros2 topic echo /sim/cmd_vel     # same values as /cmd_vel → State Sync
ros2 service call /get_twin_status std_srvs/srv/Trigger
```

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
| **Nodes** | `twin_safety_node`, `zone_monitor_node`, `dt_logger_node`, `navigator_node` |
| **Topics** | `/scan`, `/odom`, `/cmd_vel`, `/sim/cmd_vel`, `/farm_action`, `/dt/status`, `/navigator/status` |
| **Services** | `/get_twin_status`, `/get_zone_status`, `/get_dt_status`, `/get_dt_log`, `/start_navigation`, `/stop_navigation`, `/nav_status` |

---

## Package Structure

```
farm_twin_poc/
├── farm_twin_poc/
│   ├── twin_safety_node.py    Node 2 — safety stop + state sync
│   ├── zone_monitor_node.py   Node 3 — farm zone detection
│   ├── dt_logger_node.py      Node 4 — Digital Entity logger
│   └── navigator.py           Node 5 — autonomous navigation + obstacle avoidance
├── launch/
│   ├── gazebo_twin.launch.py  Gazebo Digital Twin (lab_world.sdf)
│   ├── farm_twin.launch.py    Nodes 2+3+4+5 (full system)
│   ├── slam.launch.py         SLAM with Cartographer
│   └── navigation.launch.py  Nav2 (optional)
├── scripts/
│   └── map_to_world.py        SLAM map → Gazebo SDF world
└── worlds/
    └── lab_world.sdf          Lab room (with colored zone markers)
```

---

## Farm Zones

4 predefined zones marked as colored spheres in Gazebo:
- 🔴 **Red** = spray zones (Striga herbicide)
- 🟢 **Green** = fertilize zones (NPK application)

Coordinates are in the odom frame (metres from robot start position).
Update in **both** files before lab session:
- `farm_twin_poc/zone_monitor_node.py` → `FARM_ZONES`
- `farm_twin_poc/navigator.py` → `WAYPOINTS`

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
# Expected: dt_logger_node, navigator_node, safety_stop_node, twin_safety_node, zone_monitor_node
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
```

**Terminal 3 — Teleop:**
```bash
ros2 run turtlebot3_teleop teleop_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

- Drive toward wall with `w` → robot stops automatically (safety stop)
- Drive toward colored sphere → farm action triggers
- `ros2 topic echo /sim/cmd_vel` → same values as `/cmd_vel` (State Sync)

### Mode 2 — Autonomous navigation

**Terminal 1 — Gazebo** (same as above)

**Terminal 2 — Farm Twin nodes** (same as above)

**Terminal 3 — Start autonomous run:**
```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```

Robot navigates to each zone autonomously, avoids obstacles, pauses 2s at
each zone for spray/fertilize operation.

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
ros2 launch farm_twin_poc farm_twin.launch.py
```

> **Important:** Before running at lab, change these 3 lines in `farm_twin.launch.py`:
> ```python
> # zone_monitor_node:
> 'odom_topic': '/odom',       # was '/sim/odom'
> # navigator_node:
> 'scan_topic': '/scan',       # was '/sim/scan'
> 'odom_topic': '/odom',       # was '/sim/odom'
> ```

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

Robot drives autonomously to each farm zone, avoids obstacles, pauses 2s at
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

## B6. Demo the 3 DTAS criteria

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

## B7. All monitoring commands

```bash
ros2 topic echo /farm_action
ros2 topic echo /dt/status
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
ros2 service call /nav_status       std_srvs/srv/Trigger
```

---

## B8. Adjust zone positions

Drive robot to each zone location, check tọa độ:
```bash
ros2 topic echo /odom --once | grep -A3 "position"
```

Update same coordinates in both files:
- `farm_twin_poc/zone_monitor_node.py` → `FARM_ZONES`
- `farm_twin_poc/navigator.py` → `WAYPOINTS`

Also update `<pose>` in `worlds/lab_world.sdf` for visual markers.

```bash
git add . && git commit -m "Update zone coordinates" && git push
git pull && colcon build --packages-select farm_twin_poc && source install/setup.bash
```

---

## B9. Shutdown procedure

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

**Zone actions not triggering:** Drive within `radius` metres (default 0.2 m).
Check `ros2 topic hz /odom`.

**Robot stuck during navigation:** Automatic escape after 4s — wait or call `/stop_navigation`.

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

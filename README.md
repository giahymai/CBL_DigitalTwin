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

The rubric requires demonstrating a **Digitally Twinned Autonomous System (DTAS)**
with 3 criteria:

### ① Bi-directional Communication
**What it means:** Data flows both ways between the Physical Entity and Digital Entity.

**How we prove it:**
- PE → DE: Real robot sends `/scan` (LiDAR) and `/odom` (position) to our nodes
- DE → PE: `twin_safety_node` sends safe `/cmd_vel` back to the robot

**Demo:** `ros2 topic echo /scan --once` — shows LiDAR data arriving from real robot.
`ros2 topic echo /cmd_vel` — shows commands being sent back.

---

### ② State Synchronisation
**What it means:** The digital twin and physical entity maintain the same state.

**How we prove it:**
`twin_safety_node` publishes the **exact same command** to two topics simultaneously:
- `/cmd_vel` → real robot (Physical Entity)
- `/sim/cmd_vel` → Gazebo twin (Digital Entity simulation)

Both receive identical commands at the same instant — their states are synchronised.

**Demo:** `ros2 topic echo /sim/cmd_vel` while driving the robot.
Same values appear here as on `/cmd_vel` going to the real robot.
The Gazebo robot moves at the same time as the real robot.

---

### ③ Object/Environment Interaction
**What it means:** The system responds to its physical environment.

**How we prove it — two ways:**

**A. Safety stop:** Real robot detects an obstacle via LiDAR → Digital Entity
blocks forward motion for both real robot and Gazebo twin.

**B. Farm zone actions:** As the robot navigates the field (lab room),
`zone_monitor_node` detects when it enters a predefined farm zone and
`dt_logger_node` logs the corresponding action:
- Robot enters spray zone → DE logs: `"action": "spray"` with position + timestamp
- Robot enters fertilize zone → DE logs: `"action": "fertilize"`

This simulates the Weeding robot triggering herbicide spray and the Fertilizer
robot triggering NPK application when reaching the correct field zones.

**Demo:**
```bash
ros2 topic echo /farm_action           # watch actions trigger in real-time
ros2 service call /get_dt_log std_srvs/srv/Trigger  # show full action history
```

---

## System Architecture

```
Physical Entity (TurtleBot3 Burger)
  /scan ──────────────────────────► twin_safety_node  ──► /cmd_vel     → real robot
  /odom ──────────────────────────► zone_monitor_node      /sim/cmd_vel → Gazebo twin
                                         │
                                   /farm_action
                                         │
                                    dt_logger_node ──► /dt/status (every 5s)

Gazebo twin (via PushRosNamespace 'sim'):
  /sim/scan  → twin_safety_node
  /sim/cmd_vel ← twin_safety_node
```

---

## ROS Concepts Demonstrated

| Concept | Where |
|---|---|
| **Nodes** | `twin_safety_node`, `zone_monitor_node`, `dt_logger_node`, `navigator_node` |
| **Topics** | `/scan`, `/odom`, `/cmd_vel`, `/sim/cmd_vel`, `/farm_action`, `/dt/status` |
| **Services** | `/get_twin_status`, `/get_zone_status`, `/get_dt_status`, `/get_dt_log` |

---

## Package Structure

```
farm_twin_poc/
├── farm_twin_poc/
│   ├── safety_stop_node.py   Node 1 — safety stop, sim only (at-home testing)
│   ├── twin_safety_node.py   Node 2 — DT safety + state sync + service
│   ├── zone_monitor_node.py  Node 3 — farm zone detection + service
│   ├── dt_logger_node.py     Node 4 — Digital Entity logger + services
│   └── navigator_node.py     Node 5 — autonomous navigation to farm zones
├── launch/
│   ├── gazebo_twin.launch.py — Gazebo Digital Twin (course pattern)
│   ├── safety_stop.launch.py — Node 1 only (at-home testing)
│   ├── farm_twin.launch.py   — Nodes 2+3+4+5 (full system)
│   ├── slam.launch.py        — SLAM with Cartographer
│   └── navigation.launch.py  — Nav2 for autonomous navigation
├── scripts/
│   └── map_to_world.py       — convert SLAM map → Gazebo SDF world
└── worlds/
    └── (lab_world.sdf goes here after SLAM + conversion)
```

---

## The `source` Command

`source` loads ROS settings into the current terminal. Must run in every new terminal.

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
export ROS_DOMAIN_ID=<ROBOT_ID>
```

---

---

# PART A — AT HOME (Windows + WSL + Docker)

---

## A1. Start Docker (every session)

1. Open **Docker Desktop** on Windows
2. Open **Ubuntu** from Start Menu (not PowerShell)
3. Start the container:

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
# Expected: 5 executables (dt_logger, navigator, safety_stop, twin_safety, zone_monitor)
```

> **Build error "failed to create symbolic link":**
> ```bash
> cd /ws && rm -rf build/ install/ log/
> colcon build --packages-select farm_twin_poc && source install/setup.bash
> ```

---

## A3. Test simulation at home

Open 4 terminals (each: `docker exec` + source above).

**Terminal 1 — Gazebo Digital Twin:**
```bash
ros2 launch farm_twin_poc gazebo_twin.launch.py
```
Gazebo opens with TurtleBot3 robot. Topics `/sim/scan` and `/sim/cmd_vel` active.

**Terminal 2 — Farm Twin nodes:**
```bash
ros2 launch farm_twin_poc farm_twin.launch.py
```
Expected:
```
[twin_safety_node]:  Twin Safety Node started
[twin_safety_node]:  PUB: /cmd_vel | /sim/cmd_vel → State Synchronisation
[zone_monitor_node]: Zone Monitor Node started | 4 zones loaded
[dt_logger_node]:    Digital Twin Logger (Digital Entity) started
[navigator_node]:    Navigator Node started | 4 waypoints
```

**Terminal 3 — Teleop:**
```bash
ros2 run turtlebot3_teleop teleop_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

**Terminal 4 — Monitor DE:**
```bash
ros2 topic echo /dt/status
```

**Verify safety stop:** Drive toward wall with `w` → robot stops.

**Verify State Sync:**
```bash
ros2 topic echo /sim/cmd_vel
```
Drive robot → same values appear here and on `/cmd_vel` simultaneously.

**Verify zone detection:** Drive to (1.0, 0.0) from start position.
Terminal 2 prints:
```
[ZONE ENTRY] SPRAY → spray_zone_A
[DE LOG] SPRAY | zone=spray_zone_A | total: spray=1
```

---

## A4. Push to GitHub

```bash
cd /home/c2irr10/turtlebot3_ws/src/farm_twin_poc
find . -name "*Zone.Identifier*" -delete
git add .
git commit -m "Farm Twin PoC ready for lab"
git push
```

---

---

# PART B — LAB SESSION

**No Docker. No WSL.** Everything runs directly on Ubuntu.
Workspace: `~/turtlebot3_ws`

**ROS_DOMAIN_ID = last number of robot IP.**
Example: IP `192.168.8.40` → `ROS_DOMAIN_ID=40`

Add to every terminal in the lab:
```bash
export ROS_DOMAIN_ID=<ROBOT_ID>
```

---

## B1. Setup (first time)

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

**Terminal 1 — SSH into robot:**
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

Verify connection:
```bash
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
source /opt/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 topic list | grep scan
# Must see: /scan
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
RViz shows map being built.

**Terminal 3 — Drive robot around the room:**
```bash
export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 run turtlebot3_teleop teleop_keyboard
```
Drive slowly along all walls until full room is mapped. Grey = not scanned yet.

**Terminal 4 — Save map:**
```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 run nav2_map_server map_saver_cli -f ~/map \
    --ros-args -p map_topic:=/map
```
Creates `~/map.pgm` and `~/map.yaml`. Stop Cartographer (Ctrl+C T2).

> **Error "failed to spin map subscription":**
> Ensure `ROS_DOMAIN_ID` is set and Cartographer is still running.

---

## B4. Run the full Digital Twin

Open **5 terminals**. Each needs the full source block + `ROS_DOMAIN_ID`.

**Terminal 1 — Robot bringup** (already running, keep it).

**Terminal 2 — Gazebo Digital Twin:**
```bash
ros2 launch farm_twin_poc gazebo_twin.launch.py
```
Gazebo opens. TurtleBot3 robot is visible. Topics `/sim/scan` and `/sim/cmd_vel` active.

**Terminal 3 — Farm Twin system:**
```bash
ros2 launch farm_twin_poc farm_twin.launch.py
```

**Terminal 4 — Teleop:**
```bash
ros2 run turtlebot3_teleop teleop_keyboard \
    --ros-args -r /cmd_vel:=/cmd_vel_raw
```

**Terminal 5 — State Sync verification:**
```bash
ros2 topic echo /sim/cmd_vel
```

---

## B5. Optional: Autonomous navigation

Requires the SLAM map saved in B3.

**Terminal 3b — Nav2:**
```bash
ros2 launch farm_twin_poc navigation.launch.py map:=~/map.yaml
```
In RViz: click **"2D Pose Estimate"** → click robot's actual position on the map.

**Start autonomous navigation:**
```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```
Robot drives autonomously to each farm zone. Zone actions trigger automatically.

---

## B6. Demo the 3 DTAS criteria

### ① Bi-directional Communication
```bash
ros2 topic echo /scan --once      # LiDAR data PE → DE
ros2 topic echo /cmd_vel --once   # command DE → PE
```

### ② State Synchronisation
Keep Terminal 5 visible. Drive robot.
Both `/cmd_vel` (real robot) and `/sim/cmd_vel` (Gazebo twin) receive identical
values simultaneously. The Gazebo robot moves at the same time.

```bash
ros2 service call /get_twin_status std_srvs/srv/Trigger
```

### ③ Object/Environment Interaction
**Safety stop:** Place obstacle in front of robot → drives toward it → stops.

**Zone actions:** Drive to zone marker → Terminal 3 prints farm action.
```bash
ros2 topic echo /farm_action
ros2 service call /get_dt_log std_srvs/srv/Trigger   # full action history
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
```

---

## B8. Adjust zone positions

Before the lab session, edit `farm_twin_poc/zone_monitor_node.py` AND
`farm_twin_poc/navigator_node.py` — keep coordinates **identical** in both files.

```python
FARM_ZONES / WAYPOINTS = [
    {'name': 'spray_zone_A',     'x':  1.0, 'y':  0.0, ...},  # 1m forward
    {'name': 'fertilize_zone_B', 'x':  0.0, 'y':  1.0, ...},  # 1m left
    {'name': 'spray_zone_C',     'x': -1.0, 'y':  0.0, ...},  # 1m back
    {'name': 'fertilize_zone_D', 'x':  0.0, 'y': -1.0, ...},  # 1m right
]
```

After editing:
```bash
git add . && git commit -m "Update zone coordinates" && git push
# Lab laptop:
git pull && colcon build --packages-select farm_twin_poc && source install/setup.bash
```

---

## B9. Shutdown procedure

1. `Ctrl+C` — Terminal 4 (teleop)
2. `Ctrl+C` — Terminal 3 (farm twin nodes)
3. `Ctrl+C` — Terminal 2 (Gazebo)
4. Terminal 1 (SSH): `sudo shutdown now` — wait for connection to drop
5. Flip the physical power switch on the robot

---

## Troubleshooting

**`ros2` not found:** `source /opt/ros/jazzy/setup.bash`

**Package not found:** `source ~/turtlebot3_ws/install/setup.bash`
(at home: `source /ws/install/setup.bash`)

**`turtlebot3_description` not found:** Add `source /opt/turtlebot3_ws/install/setup.bash`

**No `/scan` from robot:** Check Terminal 1 bringup and `ROS_DOMAIN_ID`.

**"failed to spin map subscription":**
```bash
export ROS_DOMAIN_ID=<ROBOT_ID>
ros2 run nav2_map_server map_saver_cli -f ~/map --ros-args -p map_topic:=/map
```

**Zone actions not triggering:** Drive within `radius` metres of zone (default 0.35 m).
Check: `ros2 topic hz /odom`

**Build error "failed to create symbolic link":**
```bash
cd ~/turtlebot3_ws && rm -rf build/ install/ log/
colcon build --packages-select farm_twin_poc && source install/setup.bash
```

# Human-Aware Navigation

ROS2 navigation system with LiDAR-based human detection and tracking for safe robot navigation in crowded indoor spaces.

## Prerequisites

- Webots R2023b or later (native install)
- Docker (for macOS/Windows) or ROS2 Humble (for Linux native)
- Foxglove Studio (optional, for visualization)

## Build Docker Image

```bash
cd <project-root>
docker build -t ros2-webots .
```

---

## macOS Setup (Docker + Native Webots)

Webots runs natively on Mac for GPU acceleration, ROS2 runs in Docker container, communication via TCP bridge.

**Terminal 1: Webots Bridge Server**
```bash
cd <project-root>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
```

**Terminal 2: Docker Container**
```bash
cd <project-root>
docker run --rm -it \
    -v ./webots_shared:/shared \
    -v ./test/human_nav_ws:/ros2_ws \
    -e WEBOTS_SHARED_FOLDER=$(pwd)/webots_shared:/shared \
    -p 8765:8765 \
    -p 2000:2000 \
    ros2-webots bash
```

Inside Docker, build the workspace:
```bash
source /opt/ros/humble/setup.bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

For additional terminals into the running container:
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && <command>"
```

---

## Linux Setup (Native ROS2)

No Docker needed - run everything natively.

```bash
cd ~/human_nav_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## Teleop Mode

Manual keyboard control for testing robot movement.

### macOS

**Terminal 1: Bridge Server**
```bash
cd <project-root>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
```

**Terminal 2: Docker + Robot Driver**
```bash
cd <project-root>
docker run --rm -it \
    -v ./webots_shared:/shared \
    -v ./test/human_nav_ws:/ros2_ws \
    -e WEBOTS_SHARED_FOLDER=$(pwd)/webots_shared:/shared \
    -p 8765:8765 -p 2000:2000 \
    ros2-webots bash
```
Inside Docker:
```bash
source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch human_aware_nav robot_launch.py
```

**Terminal 3: Teleop (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

### Linux

**Terminal 1: Robot Driver**
```bash
cd ~/human_nav_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch human_aware_nav robot_launch.py
```

**Terminal 2: Teleop**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controls: `i` = forward, `k` = stop, `j` = turn left, `l` = turn right, `,` = backward

---

## SLAM Mapping Mode

Create a new map of the environment. Use the static world (no pedestrians) for clean mapping.
Note: We use slam_toolbox, the built-in ROS 2 SLAM package, rather than developing a SLAM algorithm from scratch.

### macOS

**Terminal 1: Bridge Server**
```bash
cd <project-root>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
```

**Terminal 2: Docker + Robot Driver**
```bash
cd <project-root>
docker run --rm -it \
    -v ./webots_shared:/shared \
    -v ./test/human_nav_ws:/ros2_ws \
    -e WEBOTS_SHARED_FOLDER=$(pwd)/webots_shared:/shared \
    -p 8765:8765 -p 2000:2000 \
    ros2-webots bash
```
Inside Docker:
```bash
source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch human_aware_nav robot_launch.py world:=human_env_static.wbt
```

**Terminal 3: SLAM (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/ros2_ws/src/human_aware_nav/config/slam_params.yaml"
```

**Terminal 4: Teleop (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

**Terminal 5: Foxglove Bridge (Docker exec, optional)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
```

Open Foxglove Studio, connect to `ws://localhost:8765`, add Map panel.

### Linux

**Terminal 1: Robot Driver**
```bash
cd ~/human_nav_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch human_aware_nav robot_launch.py world:=human_env_static.wbt
```

**Terminal 2: SLAM**
```bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=~/human_nav_ws/src/human_aware_nav/config/slam_params.yaml
```

**Terminal 3: Teleop**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 4: Foxglove Bridge (optional)**
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### Saving the Map

macOS (Docker exec):
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/human_aware_nav/maps/<map_name>"
```

Linux:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/human_nav_ws/src/human_aware_nav/maps/<map_name>
```

### World Files

| World File | Description |
|------------|-------------|
| `human_env.wbt` | Full environment with 8 walking pedestrians (default) |
| `human_env_static.wbt` | Static environment without pedestrians (for SLAM) |

Use `world:=<filename>` argument to select:
```bash
ros2 launch human_aware_nav robot_launch.py world:=human_env_static.wbt
```

---

## Demo Launch Sequence

Full human-aware navigation with pedestrian detection and tracking.

### Quick Launch (macOS)

Use `launch_demo.sh` to start all side processes automatically after Docker is running.

**Terminal 1: Bridge Server**
```bash
cd <project-root>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
```

**Terminal 2: Docker + Robot Driver**
```bash
cd <project-root>
docker run --rm -it \
    -v ./webots_shared:/shared \
    -v ./test/human_nav_ws:/ros2_ws \
    -e WEBOTS_SHARED_FOLDER=$(pwd)/webots_shared:/shared \
    -p 8765:8765 -p 2000:2000 \
    ros2-webots bash
```
Inside Docker:
```bash
source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch human_aware_nav robot_launch.py
```

**Terminal 3: Launch All Side Processes**
```bash
cd <project-root>
./launch_demo.sh
```

**Terminal 4: Send Navigation Goal**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -2.6, z: 0.0}, orientation: {w: 1.0}}}}\""
```

### Manual Launch - macOS (8 Terminals, +1 Optional)

**Terminal 1: Bridge Server**
```bash
cd <project-root>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
```

**Terminal 2: Robot Driver (Docker)**
```bash
docker run --rm -it \
    -v ./webots_shared:/shared \
    -v ./test/human_nav_ws:/ros2_ws \
    -e WEBOTS_SHARED_FOLDER=$(pwd)/webots_shared:/shared \
    -p 8765:8765 -p 2000:2000 \
    ros2-webots bash
```
Inside Docker:
```bash
source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch human_aware_nav robot_launch.py
```

**Terminal 3: Human Detector (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run human_aware_nav human_detector"
```

**Terminal 4: Kalman Tracker (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run human_aware_nav kalman_tracker"
```

**Terminal 5: Scan Filter (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run human_aware_nav scan_filter"
```

**Terminal 6: Nav2 Stack (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && source install/setup.bash && ros2 launch human_aware_nav nav2_launch.py"
```

**Terminal 7: Adaptive Safety (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run human_aware_nav adaptive_safety"
```

**Terminal 8: Foxglove Bridge (Docker exec, optional)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
```
Connect Foxglove Studio to `ws://localhost:8765`.

**Terminal 9: Send Navigation Goal (Docker exec)**
```bash
docker exec -it $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -2.6, z: 0.0}, orientation: {w: 1.0}}}}\""
```

### Linux (7 Terminals)

**Terminal 1: Robot Driver**
```bash
ros2 launch human_aware_nav robot_launch.py
```

**Terminal 2: Human Detector**
```bash
ros2 run human_aware_nav human_detector
```

**Terminal 3: Kalman Tracker**
```bash
ros2 run human_aware_nav kalman_tracker
```

**Terminal 4: Scan Filter**
```bash
ros2 run human_aware_nav scan_filter
```

**Terminal 5: Nav2 Stack**
```bash
ros2 launch human_aware_nav nav2_launch.py
```

**Terminal 6: Adaptive Safety**
```bash
ros2 run human_aware_nav adaptive_safety
```

**Terminal 7: Send Navigation Goal**
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -2.6, z: 0.0}, orientation: {w: 1.0}}}}"
```

---

## Troubleshooting

**Port 2000 already in use**
```bash
lsof -ti:2000 | xargs kill -9
```

**Stop all Docker containers**
```bash
docker stop $(docker ps -q)
```

**Kill everything (macOS)**
```bash
docker stop $(docker ps -q) 2>/dev/null
lsof -ti:2000 | xargs kill -9 2>/dev/null
pkill -f "local_simulation_server"
pkill -f Webots
```

**Transform errors / goal succeeds immediately**
- Ensure driver is running and publishing TF before starting Nav2.
- Check Webots is playing (not paused).

**Robot not avoiding pedestrians**
- Verify human_detector, kalman_tracker, and adaptive_safety are all running.
- Check `/tracked_objects` topic has data: `ros2 topic echo /tracked_objects`

---

## Key Topics

| Topic | Description |
|-------|-------------|
| `/scan` | Raw LiDAR data |
| `/scan_filtered` | LiDAR with moving pedestrians removed (for AMCL) |
| `/detected_humans` | Clustered detections from human_detector |
| `/tracked_objects` | Kalman-filtered tracks with velocities |
| `/cmd_vel_raw` | Nav2 velocity commands (before safety filter) |
| `/cmd_vel` | Final velocity commands (after adaptive_safety) |

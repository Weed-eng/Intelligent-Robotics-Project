#!/bin/bash
# automated launch script for human-aware navigation demo
# uses tmux to manage all terminals in one window

set -e

PROJECT_DIR="/Users/da_ya_pavl/Documents/GitHub/Intelligent-Robotics-Project"
SESSION="human_nav"

# check dependencies
if ! command -v tmux &> /dev/null; then
    echo "tmux not found. install with: brew install tmux"
    exit 1
fi

if ! command -v docker &> /dev/null; then
    echo "docker not found. install Docker Desktop first"
    exit 1
fi

# kill existing session if present
tmux kill-session -t $SESSION 2>/dev/null || true

# stop any running containers
docker stop $(docker ps -q) 2>/dev/null || true

# kill any existing webots bridge
lsof -ti:2000 | xargs kill -9 2>/dev/null || true
pkill -f "local_simulation_server" 2>/dev/null || true

echo "starting human-aware navigation demo..."
sleep 1

# create tmux session with first window (webots bridge)
tmux new-session -d -s $SESSION -n "webots"
tmux send-keys -t $SESSION:webots "cd $PROJECT_DIR && export WEBOTS_HOME=/Applications/Webots.app && python3 local_simulation_server.py" Enter

sleep 2

# window 2: docker container (stays interactive for colcon build)
tmux new-window -t $SESSION -n "docker"
tmux send-keys -t $SESSION:docker "cd $PROJECT_DIR && docker run --rm -it -v ./webots_shared:/shared -v ./test/human_nav_ws:/ros2_ws -e WEBOTS_SHARED_FOLDER=\$(pwd)/webots_shared:/shared -p 8765:8765 -p 2000:2000 ros2-webots bash" Enter

echo ""
echo "=========================================="
echo "  MANUAL STEPS REQUIRED:"
echo "=========================================="
echo ""
echo "1. Switch to tmux session: tmux attach -t $SESSION"
echo ""
echo "2. In 'docker' window, run:"
echo "   source /opt/ros/humble/setup.bash"
echo "   cd /ros2_ws && colcon build --symlink-install"
echo "   source install/setup.bash"
echo "   ros2 launch human_aware_nav robot_launch.py"
echo ""
echo "3. CLICK PLAY in Webots when it opens"
echo ""
echo "4. Once running, open new tmux windows (Ctrl-b c) and run:"
echo ""
echo "   # human detector"
echo "   docker exec -it \$(docker ps -q) bash -c 'source /ros2_ws/install/setup.bash && ros2 run human_aware_nav human_detector'"
echo ""
echo "   # scan filter"
echo "   docker exec -it \$(docker ps -q) bash -c 'source /ros2_ws/install/setup.bash && ros2 run human_aware_nav scan_filter'"
echo ""
echo "   # nav2"
echo "   docker exec -it \$(docker ps -q) bash -c 'source /ros2_ws/install/setup.bash && ros2 launch human_aware_nav nav2_launch.py'"
echo ""
echo "   # kalman tracker"
echo "   docker exec -it \$(docker ps -q) bash -c 'source /ros2_ws/install/setup.bash && ros2 run human_aware_nav kalman_tracker'"
echo ""
echo "   # adaptive safety"
echo "   docker exec -it \$(docker ps -q) bash -c 'source /ros2_ws/install/setup.bash && ros2 run human_aware_nav adaptive_safety'"
echo ""
echo "   # foxglove bridge"
echo "   docker exec -it \$(docker ps -q) bash -c 'source /opt/ros/humble/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'"
echo ""
echo "5. Open Foxglove Studio and connect to ws://localhost:8765"
echo ""
echo "6. Send navigation goal:"
echo "   docker exec -it \$(docker ps -q) bash -c \"source /opt/ros/humble/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\\"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -2.6, z: 0.0}, orientation: {w: 1.0}}}}\\\"\""
echo ""
echo "=========================================="
echo ""
echo "tmux commands:"
echo "  attach: tmux attach -t $SESSION"
echo "  switch windows: Ctrl-b n (next) / Ctrl-b p (prev)"
echo "  new window: Ctrl-b c"
echo "  kill session: tmux kill-session -t $SESSION"
echo ""

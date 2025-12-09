#!/bin/bash
# launches all side processes for human-aware navigation
# assumes docker is already running with robot_launch.py

set -e

echo "launching side processes..."

# human detector
docker exec -d $(docker ps -q) bash -c "source /ros2_ws/install/setup.bash && ros2 run human_aware_nav human_detector" &

# scan filter
docker exec -d $(docker ps -q) bash -c "source /ros2_ws/install/setup.bash && ros2 run human_aware_nav scan_filter" &

# kalman tracker
docker exec -d $(docker ps -q) bash -c "source /ros2_ws/install/setup.bash && ros2 run human_aware_nav kalman_tracker" &

# adaptive safety
docker exec -d $(docker ps -q) bash -c "source /ros2_ws/install/setup.bash && ros2 run human_aware_nav adaptive_safety" &

# nav2
docker exec -d $(docker ps -q) bash -c "source /ros2_ws/install/setup.bash && ros2 launch human_aware_nav nav2_launch.py" &

# foxglove bridge
docker exec -d $(docker ps -q) bash -c "source /opt/ros/humble/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765" &

sleep 2
echo "all processes launched"
echo ""
echo "send nav goal with:"
echo "docker exec -it \$(docker ps -q) bash -c \"ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\\"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: -2.6, z: 0.0}, orientation: {w: 1.0}}}}\\\"\""

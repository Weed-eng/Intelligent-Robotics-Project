#!/bin/bash
# launches all side processes for human-aware navigation
# assumes docker is already running with robot_launch.py
# starts processes in dependency order with delays

set -e

CONTAINER=$(docker ps -q)
if [ -z "$CONTAINER" ]; then
    echo "error: no docker container running"
    exit 1
fi

echo "launching side processes..."

# 1. human detector - detects humans from /scan
echo "  [1/6] human_detector..."
docker exec -d $CONTAINER bash -c "source /ros2_ws/install/setup.bash && ros2 run human_aware_nav human_detector"
sleep 2

# 2. kalman tracker - tracks detected humans, publishes /tracked_objects
echo "  [2/6] kalman_tracker..."
docker exec -d $CONTAINER bash -c "source /ros2_ws/install/setup.bash && ros2 run human_aware_nav kalman_tracker"
sleep 2

# 3. scan filter - removes moving pedestrians from scan for AMCL
echo "  [3/6] scan_filter..."
docker exec -d $CONTAINER bash -c "source /ros2_ws/install/setup.bash && ros2 run human_aware_nav scan_filter"
sleep 2

# 4. nav2 - needs scan_filtered for AMCL localization
echo "  [4/6] nav2..."
docker exec -d $CONTAINER bash -c "source /ros2_ws/install/setup.bash && ros2 launch human_aware_nav nav2_launch.py"
sleep 10

# 5. adaptive safety - needs tracked_objects and cmd_vel_raw from nav2
echo "  [5/6] adaptive_safety..."
docker exec -d $CONTAINER bash -c "source /ros2_ws/install/setup.bash && ros2 run human_aware_nav adaptive_safety"
sleep 2

# 6. foxglove bridge - visualization
echo "  [6/6] foxglove_bridge..."
docker exec -d $CONTAINER bash -c "source /opt/ros/humble/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
sleep 1

echo ""
echo "all processes launched"
echo ""

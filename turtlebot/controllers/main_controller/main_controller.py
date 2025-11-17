# main controller - integration hub for human-aware navigation
# combines slam, lidar detection, kalman predictor, adaptive safety, motion planner

import sys
import os

# add parent dir to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from motion_planner import MotionPlannerController


def main():
    # runs goal-seeking p-controller test
    # todo: integrate slam, lidar, kalman, safety modules

    print("=" * 60)
    print("MAIN CONTROLLER - Human-Aware Navigation System")
    print("=" * 60)
    print("Status: Testing P-controller goal seeking")
    print("-" * 60)

    # init motion planner
    motion_controller = MotionPlannerController()

    # define test waypoints (x, y)
    waypoints = [
        (1.0, 0.0),   # straight ahead
        (1.0, 1.0),   # turn left, forward
        (0.0, 1.0),   # turn left, forward
        (0.0, 0.0),   # return to origin (square path)
    ]

    # run goal-seeking test
    motion_controller.run_goal_seeking_test(waypoints)


if __name__ == "__main__":
    main()

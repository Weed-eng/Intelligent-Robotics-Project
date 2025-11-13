# Main Controller

This controller implements the **full human-aware navigation pipeline** for the robot.  
It integrates all perception, prediction, safety, and planning modules to produce the final motor commands in Webots.

The processing flow is:

```
          LiDAR
            ↓
     Detection/Clustering
            ↓
      Kalman Predictor  
            ↓
     Adaptive Safety Logic ← uses SLAM pose
            ↓
            DWA ← uses SLAM pose
            ↓
        Motor Commands

```

---

## Purpose

The main controller coordinates every module in the system:

- Runs SLAM/localization to get robot pose  
- Reads LiDAR detections from the perception module  
- Maintains Kalman filters for tracking humans/objects  
- Computes dynamic safety radii and risk levels  
- Updates DWA parameters based on predictions  
- Selects the safest velocity command  
- Sends motor commands to the robot  
- Logs collisions and minimum clearance for evaluation  

This is the controller used in all three experiments.

---

## How to Use

### 1. Build (if C/C++)
```bash
make
```

Python controllers require no build step.

---

### 2. Assign Controller in Webots

1. Select the robot in the Scene Tree  
2. Set the controller field to:

```
main_controller
```

3. Run the simulation.

---

## Experiment Modes

The controller supports all three required evaluation modes:

### 1. Baseline DWA
- No predictions  
- No adaptive safety  
- Standard obstacle avoidance  

### 2. Kalman-Enhanced DWA
- Human motion prediction enabled  
- DWA still uses fixed safety parameters  

### 3. Adaptive Safety DWA
- Dynamic safety radius  
- Speed scaling based on risk  
- Predictive human-aware avoidance  
- Full intelligent navigation mode  

---

## Data Logging

The main controller logs:

- collisions  
- minimum distance to humans  
- robot velocity commands  
- prediction usage statistics  

Logs are saved to:

```
/data/baseline_runs/
/data/kalman_runs/
/data/adaptive_runs/
```

These logs are used for final comparison and report figures.

---

## Modules Used

The main controller depends on the following project modules:

- `lidar_detector/` — LiDAR raw detection  
- `kalman_predictor/` — human motion tracking  
- `adaptive_safety/` — dynamic safety logic  
- `dwa_planner/` — trajectory evaluation  
- `slam_localization/` — pose estimation  
- `utils/` — math and logging helpers  

---

## Contributor

**Waleed Tariq**

Responsibilities:
- Full pipeline integration  
- Kalman predictor + adaptive safety logic integration  
- Logging and experiment control  
- Motion coordination between modules  
- Ensuring compatibility between subsystems  

---

## Notes

- This controller should be assigned in every experiment world.  
- If SLAM output is unavailable, odometry fallback is used.  
- Ensure LiDAR device names match those in the robot PROTO.

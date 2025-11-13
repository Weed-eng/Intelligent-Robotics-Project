DWA Local Planner

Implements a standard Dynamic Window Approach planner:

samples velocity commands

simulates trajectories

evaluates goal, heading, obstacle, and smoothness costs

selects the best (v, ω) at each timestep

In experiment mode 1 (baseline), the planner runs without prediction/safety adaptation.

Run

Used internally by main_controller.

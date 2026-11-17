# Point-to-point navigation - results

Source run(s): tour_20261117_055835  
Goals: **6**, reached: **6**, success rate: **100 %**

## Per-goal results

| Test | Route | Result | Time [s] | Driven [m] | Planned [m] | Efficiency | Pos. error [m] | Yaw error [deg] | Recoveries | Min clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|
| L1 | current -> lab | SUCCEEDED | 23.6 | 3.77 | 3.85 | 1.02 | 0.036 | 1.1 | 3 | 0.37 |
| L2 | current -> office | SUCCEEDED | 25.5 | 4.04 | 4.27 | 1.06 | 0.051 | 3.6 | 2 | 0.25 |
| L3 | current -> meeting_room | SUCCEEDED | 42.4 | 6.73 | 6.88 | 1.02 | 0.046 | 2.8 | 2 | 0.35 |
| L4 | current -> storage | SUCCEEDED | 52.9 | 8.09 | 8.30 | 1.03 | 0.102 | 4.4 | 10 | 0.31 |
| L5 | current -> kitchen | SUCCEEDED | 67.4 | 11.42 | 11.22 | 0.98 | 0.056 | 4.1 | 9 | 0.28 |
| L6 | current -> reception | SUCCEEDED | 35.6 | 3.65 | 3.86 | 1.06 | 0.082 | 8.3 | 7 | 0.31 |

## Aggregate (successful goals)

| Metric | Mean | Std |
|---|---|---|
| Final position error [m] | 0.062 | 0.025 |
| Final yaw error [deg] | 4.0 | 2.4 |
| Navigation time [s] | 41.2 | 16.8 |
| Driven path length [m] | 6.283 | 3.104 |
| Planned path length [m] | 6.396 | 2.986 |
| Straight-line distance [m] | 3.778 | 1.083 |
| Path efficiency (planned / driven) | 1.028 | 0.028 |
| Average speed [m/s] | 0.150 | 0.024 |
| Recovery behaviours per goal | 5.500 | 3.619 |
| Minimum clearance to obstacles [m] | 0.314 | 0.045 |
| Localisation error (AMCL vs truth) [m] | 0.064 | 0.024 |

## Definitions

- **Position error** E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2): goal vs final pose. In Gazebo Classic the final pose is the simulator ground truth (/odom = world pose); otherwise the AMCL estimate is used (column `ground_truth` in trials.csv).
- **Yaw error**: |wrap(yaw_g - yaw_f)|.
- **Driven path length** L = sum of distances between consecutive recorded poses.
- **Planned path length**: length of the Nav2 global plan computed before the robot moves.
- **Path efficiency** = planned length / driven length (1.0 = drove exactly the planned distance).
- **Success rate** = goals reached / goals attempted x 100 %.
- **Min clearance** = closest LiDAR range during the run minus the robot radius (0.105 m).
- Goals are accepted by Nav2 within xy_goal_tolerance / yaw_goal_tolerance (config/nav2_params_*.yaml), so small final errors are expected by design.

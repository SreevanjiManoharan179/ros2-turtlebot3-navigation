# Point-to-point navigation - results

Source run(s): tour_20261117_125851  
Goals: **6**, reached: **6**, success rate: **100 %**

## Per-goal results

| Test | Route | Result | Time [s] | Driven [m] | Planned [m] | Efficiency | Pos. error [m] | Yaw error [deg] | Recoveries | Min clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|
| L1 | current -> lab | SUCCEEDED | 23.6 | 3.72 | 3.88 | 1.04 | 0.039 | 5.1 | 0 | 0.36 |
| L2 | current -> office | SUCCEEDED | 26.0 | 4.07 | 4.33 | 1.06 | 0.069 | 4.5 | 0 | 0.25 |
| L3 | current -> meeting_room | SUCCEEDED | 41.9 | 6.68 | 6.82 | 1.02 | 0.035 | 4.6 | 0 | 0.35 |
| L4 | current -> storage | SUCCEEDED | 44.8 | 8.01 | 8.39 | 1.05 | 0.061 | 1.6 | 0 | 0.33 |
| L5 | current -> kitchen | SUCCEEDED | 66.5 | 11.43 | 11.55 | 1.01 | 0.011 | 5.8 | 0 | 0.26 |
| L6 | current -> reception | SUCCEEDED | 25.3 | 3.68 | 3.96 | 1.08 | 0.037 | 5.4 | 0 | 0.33 |

## Aggregate (successful goals)

| Metric | Mean | Std |
|---|---|---|
| Final position error [m] | 0.042 | 0.021 |
| Final yaw error [deg] | 4.5 | 1.5 |
| Navigation time [s] | 38.0 | 16.7 |
| Driven path length [m] | 6.266 | 3.094 |
| Planned path length [m] | 6.487 | 3.071 |
| Straight-line distance [m] | 3.776 | 1.056 |
| Path efficiency (planned / driven) | 1.043 | 0.025 |
| Average speed [m/s] | 0.162 | 0.012 |
| Recovery behaviours per goal | 0.000 | 0.000 |
| Minimum clearance to obstacles [m] | 0.311 | 0.046 |
| Localisation error (AMCL vs truth) [m] | 0.036 | 0.015 |

## Definitions

- **Position error** E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2): goal vs final pose. In Gazebo Classic the final pose is the simulator ground truth (/odom = world pose); otherwise the AMCL estimate is used (column `ground_truth` in trials.csv).
- **Yaw error**: |wrap(yaw_g - yaw_f)|.
- **Driven path length** L = sum of distances between consecutive recorded poses.
- **Planned path length**: length of the Nav2 global plan computed before the robot moves.
- **Path efficiency** = planned length / driven length (1.0 = drove exactly the planned distance).
- **Success rate** = goals reached / goals attempted x 100 %.
- **Min clearance** = closest LiDAR range during the run minus the robot radius (0.105 m).
- Goals are accepted by Nav2 within xy_goal_tolerance / yaw_goal_tolerance (config/nav2_params_*.yaml), so small final errors are expected by design.

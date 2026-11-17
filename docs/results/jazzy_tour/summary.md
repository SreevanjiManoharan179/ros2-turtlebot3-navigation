# Point-to-point navigation - results

Source run(s): tour_20261117_122738  
Goals: **6**, reached: **6**, success rate: **100 %**

## Per-goal results

| Test | Route | Result | Time [s] | Driven [m] | Planned [m] | Efficiency | Pos. error [m] | Yaw error [deg] | Recoveries | Min clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|
| L1 | current -> lab | SUCCEEDED | 23.6 | 3.76 | 3.87 | 1.03 | 0.055 | 1.9 | 0 | 0.37 |
| L2 | current -> office | SUCCEEDED | 32.7 | 4.06 | 4.27 | 1.05 | 0.088 | 14.3 | 0 | 0.25 |
| L3 | current -> meeting_room | SUCCEEDED | 41.8 | 6.78 | 6.71 | 0.99 | 0.051 | 0.4 | 0 | 0.36 |
| L4 | current -> storage | SUCCEEDED | 44.7 | 8.02 | 8.15 | 1.02 | 0.049 | 5.2 | 0 | 0.32 |
| L5 | current -> kitchen | SUCCEEDED | 66.7 | 11.41 | 11.43 | 1.00 | 0.044 | 0.5 | 0 | 0.29 |
| L6 | current -> reception | SUCCEEDED | 46.7 | 3.77 | 4.00 | 1.06 | 0.041 | 0.5 | 1 | 0.33 |

## Aggregate (successful goals)

| Metric | Mean | Std |
|---|---|---|
| Final position error [m] | 0.055 | 0.017 |
| Final yaw error [deg] | 3.8 | 5.5 |
| Navigation time [s] | 42.7 | 14.6 |
| Driven path length [m] | 6.301 | 3.071 |
| Planned path length [m] | 6.405 | 3.004 |
| Straight-line distance [m] | 3.759 | 1.046 |
| Path efficiency (planned / driven) | 1.025 | 0.028 |
| Average speed [m/s] | 0.146 | 0.037 |
| Recovery behaviours per goal | 0.167 | 0.408 |
| Minimum clearance to obstacles [m] | 0.319 | 0.045 |

## Definitions

- **Position error** E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2): goal vs final pose. In Gazebo Classic the final pose is the simulator ground truth (/odom = world pose); otherwise the AMCL estimate is used (column `ground_truth` in trials.csv).
- **Yaw error**: |wrap(yaw_g - yaw_f)|.
- **Driven path length** L = sum of distances between consecutive recorded poses.
- **Planned path length**: length of the Nav2 global plan computed before the robot moves.
- **Path efficiency** = planned length / driven length (1.0 = drove exactly the planned distance).
- **Success rate** = goals reached / goals attempted x 100 %.
- **Min clearance** = closest LiDAR range during the run minus the robot radius (0.105 m).
- Goals are accepted by Nav2 within xy_goal_tolerance / yaw_goal_tolerance (config/nav2_params_*.yaml), so small final errors are expected by design.

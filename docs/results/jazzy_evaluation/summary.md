# Point-to-point navigation - results

Source run(s): evaluation_20261117_123224  
Goals: **8**, reached: **8**, success rate: **100 %**

## Per-goal results

| Test | Route | Result | Time [s] | Driven [m] | Planned [m] | Efficiency | Pos. error [m] | Yaw error [deg] | Recoveries | Min clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|
| T1 | reception -> lab | SUCCEEDED | 24.7 | 3.88 | 13.11 | 3.38 | 0.059 | 1.8 | 0 | 0.32 |
| T2 | lab -> office | SUCCEEDED | 26.3 | 4.08 | 4.28 | 1.05 | 0.056 | 1.2 | 0 | 0.25 |
| T3 | office -> meeting_room | SUCCEEDED | 42.0 | 6.71 | 6.84 | 1.02 | 0.055 | 2.5 | 0 | 0.36 |
| T4 | meeting_room -> storage | SUCCEEDED | 44.7 | 8.00 | 8.32 | 1.04 | 0.055 | 3.9 | 0 | 0.32 |
| T5 | storage -> kitchen | SUCCEEDED | 65.6 | 11.37 | 11.31 | 0.99 | 0.025 | 0.6 | 0 | 0.27 |
| T6 | kitchen -> reception | SUCCEEDED | 28.6 | 3.69 | 4.14 | 1.12 | 0.076 | 5.0 | 0 | 0.31 |
| T7 | reception -> meeting_room | SUCCEEDED | 57.8 | 10.76 | 11.28 | 1.05 | 0.041 | 2.8 | 0 | 0.28 |
| T8 | corridor_west -> corridor_east (+obstacle) | SUCCEEDED | 76.8 | 13.84 | 10.83 | 0.78 | 0.053 | 1.7 | 0 | 0.30 |

## Aggregate (successful goals)

| Metric | Mean | Std |
|---|---|---|
| Final position error [m] | 0.052 | 0.015 |
| Final yaw error [deg] | 2.4 | 1.4 |
| Navigation time [s] | 45.8 | 19.4 |
| Driven path length [m] | 7.789 | 3.879 |
| Planned path length [m] | 8.764 | 3.410 |
| Straight-line distance [m] | 5.107 | 2.747 |
| Path efficiency (planned / driven) | 1.305 | 0.845 |
| Average speed [m/s] | 0.165 | 0.019 |
| Recovery behaviours per goal | 0.000 | 0.000 |
| Minimum clearance to obstacles [m] | 0.300 | 0.034 |

## Definitions

- **Position error** E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2): goal vs final pose. In Gazebo Classic the final pose is the simulator ground truth (/odom = world pose); otherwise the AMCL estimate is used (column `ground_truth` in trials.csv).
- **Yaw error**: |wrap(yaw_g - yaw_f)|.
- **Driven path length** L = sum of distances between consecutive recorded poses.
- **Planned path length**: length of the Nav2 global plan computed before the robot moves.
- **Path efficiency** = planned length / driven length (1.0 = drove exactly the planned distance).
- **Success rate** = goals reached / goals attempted x 100 %.
- **Min clearance** = closest LiDAR range during the run minus the robot radius (0.105 m).
- Goals are accepted by Nav2 within xy_goal_tolerance / yaw_goal_tolerance (config/nav2_params_*.yaml), so small final errors are expected by design.

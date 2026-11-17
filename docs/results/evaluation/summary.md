# Point-to-point navigation - results

Source run(s): evaluation_20261117_120245  
Goals: **8**, reached: **8**, success rate: **100 %**

## Per-goal results

| Test | Route | Result | Time [s] | Driven [m] | Planned [m] | Efficiency | Pos. error [m] | Yaw error [deg] | Recoveries | Min clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|
| T1 | reception -> lab | SUCCEEDED | 23.5 | 3.71 | 3.88 | 1.04 | 0.032 | 5.0 | 0 | 0.36 |
| T2 | lab -> office | SUCCEEDED | 25.9 | 4.06 | 4.26 | 1.05 | 0.066 | 2.1 | 0 | 0.25 |
| T3 | office -> meeting_room | SUCCEEDED | 42.1 | 6.68 | 6.82 | 1.02 | 0.030 | 5.6 | 0 | 0.37 |
| T4 | meeting_room -> storage | SUCCEEDED | 45.7 | 8.08 | 8.50 | 1.05 | 0.053 | 4.0 | 0 | 0.32 |
| T5 | storage -> kitchen | SUCCEEDED | 66.1 | 11.41 | 11.54 | 1.01 | 0.011 | 6.5 | 0 | 0.27 |
| T6 | kitchen -> reception | SUCCEEDED | 25.3 | 3.68 | 4.00 | 1.09 | 0.047 | 7.4 | 0 | 0.33 |
| T7 | reception -> meeting_room | SUCCEEDED | 56.7 | 10.78 | 11.27 | 1.05 | 0.014 | 3.7 | 0 | 0.27 |
| T8 | corridor_west -> corridor_east (+obstacle) | SUCCEEDED | 75.9 | 14.24 | 11.05 | 0.78 | 0.038 | 6.4 | 0 | 0.25 |

## Aggregate (successful goals)

| Metric | Mean | Std |
|---|---|---|
| Final position error [m] | 0.036 | 0.019 |
| Final yaw error [deg] | 5.1 | 1.8 |
| Navigation time [s] | 45.2 | 19.9 |
| Driven path length [m] | 7.830 | 4.006 |
| Planned path length [m] | 7.665 | 3.388 |
| Straight-line distance [m] | 5.125 | 2.749 |
| Path efficiency (planned / driven) | 1.011 | 0.097 |
| Average speed [m/s] | 0.168 | 0.016 |
| Recovery behaviours per goal | 0.000 | 0.000 |
| Minimum clearance to obstacles [m] | 0.303 | 0.050 |
| Localisation error (AMCL vs truth) [m] | 0.038 | 0.015 |

## Definitions

- **Position error** E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2): goal vs final pose. In Gazebo Classic the final pose is the simulator ground truth (/odom = world pose); otherwise the AMCL estimate is used (column `ground_truth` in trials.csv).
- **Yaw error**: |wrap(yaw_g - yaw_f)|.
- **Driven path length** L = sum of distances between consecutive recorded poses.
- **Planned path length**: length of the Nav2 global plan computed before the robot moves.
- **Path efficiency** = planned length / driven length (1.0 = drove exactly the planned distance).
- **Success rate** = goals reached / goals attempted x 100 %.
- **Min clearance** = closest LiDAR range during the run minus the robot radius (0.105 m).
- Goals are accepted by Nav2 within xy_goal_tolerance / yaw_goal_tolerance (config/nav2_params_*.yaml), so small final errors are expected by design.

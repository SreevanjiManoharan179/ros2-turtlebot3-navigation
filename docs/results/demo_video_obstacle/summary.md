# Point-to-point navigation - results

Source run(s): obstacle_20261117_060352  
Goals: **1**, reached: **1**, success rate: **100 %**

## Per-goal results

| Test | Route | Result | Time [s] | Driven [m] | Planned [m] | Efficiency | Pos. error [m] | Yaw error [deg] | Recoveries | Min clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|
| OB1 | corridor_west -> corridor_east (+obstacle) | SUCCEEDED | 100.7 | 15.21 | 10.95 | 0.72 | 0.033 | 1.1 | 10 | 0.30 |

## Aggregate (successful goals)

| Metric | Mean | Std |
|---|---|---|
| Final position error [m] | 0.033 | 0.000 |
| Final yaw error [deg] | 1.1 | 0.0 |
| Navigation time [s] | 100.7 | 0.0 |
| Driven path length [m] | 15.214 | 0.000 |
| Planned path length [m] | 10.949 | 0.000 |
| Straight-line distance [m] | 10.500 | 0.000 |
| Path efficiency (planned / driven) | 0.720 | 0.000 |
| Average speed [m/s] | 0.151 | 0.000 |
| Recovery behaviours per goal | 10.000 | 0.000 |
| Minimum clearance to obstacles [m] | 0.298 | 0.000 |
| Localisation error (AMCL vs truth) [m] | 0.080 | 0.000 |

## Definitions

- **Position error** E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2): goal vs final pose. In Gazebo Classic the final pose is the simulator ground truth (/odom = world pose); otherwise the AMCL estimate is used (column `ground_truth` in trials.csv).
- **Yaw error**: |wrap(yaw_g - yaw_f)|.
- **Driven path length** L = sum of distances between consecutive recorded poses.
- **Planned path length**: length of the Nav2 global plan computed before the robot moves.
- **Path efficiency** = planned length / driven length (1.0 = drove exactly the planned distance).
- **Success rate** = goals reached / goals attempted x 100 %.
- **Min clearance** = closest LiDAR range during the run minus the robot radius (0.105 m).
- Goals are accepted by Nav2 within xy_goal_tolerance / yaw_goal_tolerance (config/nav2_params_*.yaml), so small final errors are expected by design.

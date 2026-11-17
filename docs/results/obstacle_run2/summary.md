# Point-to-point navigation - results

Source run(s): obstacle_20261117_121523  
Goals: **1**, reached: **1**, success rate: **100 %**

## Per-goal results

| Test | Route | Result | Time [s] | Driven [m] | Planned [m] | Efficiency | Pos. error [m] | Yaw error [deg] | Recoveries | Min clearance [m] |
|---|---|---|---|---|---|---|---|---|---|---|
| OB1 | corridor_west -> corridor_east (+obstacle) | SUCCEEDED | 76.9 | 13.77 | 10.87 | 0.79 | 0.039 | 4.9 | 0 | 0.30 |

## Aggregate (successful goals)

| Metric | Mean | Std |
|---|---|---|
| Final position error [m] | 0.039 | 0.000 |
| Final yaw error [deg] | 4.9 | 0.0 |
| Navigation time [s] | 76.9 | 0.0 |
| Driven path length [m] | 13.769 | 0.000 |
| Planned path length [m] | 10.875 | 0.000 |
| Straight-line distance [m] | 10.500 | 0.000 |
| Path efficiency (planned / driven) | 0.790 | 0.000 |
| Average speed [m/s] | 0.179 | 0.000 |
| Recovery behaviours per goal | 0.000 | 0.000 |
| Minimum clearance to obstacles [m] | 0.297 | 0.000 |
| Localisation error (AMCL vs truth) [m] | 0.032 | 0.000 |

## Definitions

- **Position error** E_p = sqrt((x_g - x_f)^2 + (y_g - y_f)^2): goal vs final pose. In Gazebo Classic the final pose is the simulator ground truth (/odom = world pose); otherwise the AMCL estimate is used (column `ground_truth` in trials.csv).
- **Yaw error**: |wrap(yaw_g - yaw_f)|.
- **Driven path length** L = sum of distances between consecutive recorded poses.
- **Planned path length**: length of the Nav2 global plan computed before the robot moves.
- **Path efficiency** = planned length / driven length (1.0 = drove exactly the planned distance).
- **Success rate** = goals reached / goals attempted x 100 %.
- **Min clearance** = closest LiDAR range during the run minus the robot radius (0.105 m).
- Goals are accepted by Nav2 within xy_goal_tolerance / yaw_goal_tolerance (config/nav2_params_*.yaml), so small final errors are expected by design.

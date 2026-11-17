# 6. Design decisions and lessons learned

How this project differs from a typical TurtleBot3 + Nav2 tutorial setup, and why. Every row
was found while building and testing the project, not taken from a checklist.

## 6.1 Setup and workflow

| # | Typical tutorial setup | Problem (verified) | What this project does |
|---|---|---|---|
| 1 | Hand-picked (x, y) test goals in `turtlebot3_world` | Goals are easy to place outside the arena (it only spans x -2.9..2.7 m, y -2.6..2.6 m) or inside a pillar; (0, 0) *is* the central pillar, and the robot spawns at (-2.0, -0.5) | Goals are **named rooms** with checked coordinates; every goal is validated against the live map (`check_goal`) before it is sent |
| 2 | `turtlebot3_world` (hexagon with pillars) | not an indoor environment | custom **indoor office floor**: 6 rooms + corridor, 1.0 m doors, a 0.95 m shelf aisle, furniture, alternative routes |
| 3 | Goal-sender script that returns early from `__init__` when Nav2 is not up | `main()` still calls `rclpy.spin()` -> the script hangs forever | exits with a clear message and exit code; every Nav2 call has a timeout |
| 4 | `if result.status == 4` | magic number | `GoalStatus.STATUS_SUCCEEDED` |
| 5 | Metrics described as formulas only | nothing actually measures them | `metrics.py` + `mission_runner` log every metric per goal against Gazebo ground truth; `analyze_results` writes tables and plots |
| 6 | "Install `gazebo-ros-pkgs`" as the fix for a missing `/spawn_entity` | already installed; the real causes are stale `gzserver` processes, online model downloads and a missing `/usr/share/gazebo/setup.sh` | the scripts kill stale Gazebo processes, disable online model downloads, source the Gazebo setup, refuse to run as root and warn about ROS 1 in `~/.bashrc` |
| 7 | ROS apt key via `curl .../ros.key` | that key expired on 1 June 2025; the official method is the `ros2-apt-source` package | `scripts/setup.sh` uses `ros2-apt-source` and repairs old setups |
| 8 | TurtleBot3 from apt **and** `turtlebot3_simulations` from source | two copies of the same packages -> version mixes | everything ROBOTIS from apt; only this project's package is built |
| 9 | Cartographer map, goals in metres | Cartographer's map frame starts where the robot started, not at the world origin | SLAM Toolbox map in the world frame; `explore_and_map` re-anchors the map |
| 10 | TurtleBot3's Humble `burger.yaml` | still has a `recoveries_server` section (the Humble node is `behavior_server`), 0.25 m goal tolerance | complete Nav2 parameter sets for Humble and Jazzy, tuned for the Burger and this building |
| 11 | `map_saver_cli` defaults | Nav2's default `free_thresh` 0.25 reads unknown (grey 205) cells back as **free** | maps saved with `--free 0.196` |
| 12 | Manual steps in 5+ terminals | easy to forget a `source` or `use_sim_time` | `bash demo.sh` menu does everything; manual commands are still documented |
| 13 | Project path with spaces or quotes | colcon and Gazebo fail on such paths | `build.sh` builds in `~/tb3_p2p_ws` automatically |
| 14 | One ROS distribution | Ubuntu 22.04 and 24.04 ship different ROS / Gazebo generations | one code base for Humble (Gazebo Classic) **and** Jazzy (Gazebo Sim) |

## 6.2 Found while tuning

Each fix is also documented in the header of the YAML file it changed.

* **Localisation drift.** AMCL with the default laser model drifted ~0.2 m along the 12 m corridor
  and the robot got stuck in a doorway -> sharper laser model and more frequent updates (error now a
  few cm).
* **Blocked door.** A desk covered half of the Lab/Office internal door -> the door was moved in the
  layout generator.
* **Warped map.** SLAM Toolbox loop closure bent the map of this symmetric floor plan -> loop
  closure off (simulation odometry is already near-perfect).
* **Exploration goals off the costmap.** Goals outside the area seen so far cannot be planned to ->
  the exploration route always moves into seen space.
* **Map offset.** The SLAM map came out 5 cm / 2 cm off the building frame; matching its walls to
  the floor plan gives the correction (no rotation needed). `demo.sh map` now does this for every
  new map and prints how far the mapped walls are from the plan.
* **AMCL half-cell bias.** Logging every AMCL update against Gazebo ground truth over a tour showed
  a constant (-3.4, -4.4) cm error in every room. Half a cell of it is a convention mismatch: AMCL
  indexes map cells half a cell lower-left of how `map_server` and the costmaps do. Shifting the map
  origin by the measured bias cut AMCL's average error from 5.9 cm to 3.5 cm.
* **Obstacle forgotten again.** The block was erased from the costmap when the robot looked at it
  from 2.5-3.0 m (marking range 2.5 m, clearing range 3.0 m), so the route flipped back and forth ->
  obstacles are marked and cleared over the same 3.0 m range.
* **Controller stall.** DWB, TurtleBot3's default controller, stalled in front of the
  Lobby -> Kitchen door: the robot faced the door frame while the path turned ~70° to the right, DWB
  kept choosing ~zero speed and the run needed 4 recoveries (119 s) or was aborted after 13 ->
  **Regulated Pure Pursuit**, which first turns in place when the path is > 45° off: same detour in
  74 s with 0 recoveries.
* **Coasting past the goal heading.** Turning in place at 1.0 rad/s, the robot was still turning
  when the goal was accepted and coasted ~13° past the target heading -> 0.6 rad/s and a 20 Hz
  controller.
* **Needless recoveries on a busy PC.** With Gazebo and RViz rendering on a small machine, the
  behaviour tree waited only 20 ms for the planner / controller to accept a request, timed out and
  started a recovery -> 200 ms.
* **Missions on a running stack.** After one mission the robot can be anywhere. The tour used to only
  *tell* AMCL that the robot was at Reception (all six legs then failed), and a new `go_to` /
  mission process sometimes missed AMCL's latched pose and reset it to the spawn pose. Now the tour
  puts the robot at its start for real (teleport + relocalise, or drive there), and a new process
  treats a live `map -> base_footprint` transform as proof that AMCL already has a pose.
* **Jazzy: collision monitor stopping the robot.** TurtleBot3's Jazzy parameters give the LiDAR
  source a 0.2 s timeout - exactly one scan period at 5 Hz - so the monitor kept ignoring the LiDAR
  and stopping the robot -> 1.0 s.
* **Jazzy: `/clock` flood.** Gazebo Sim publishes `/clock` on every physics step; with 1 ms steps
  every ROS node received 1000 messages per second and Nav2's heartbeats timed out on a small PC ->
  4 ms steps in the Gazebo Sim world.
* **Jazzy: exploration waypoints in the wrong frame.** SLAM Toolbox's map frame starts where the
  odometry starts - the world origin in Gazebo Classic, but the spawn pose in Gazebo Sim - so on
  Jazzy every world-frame waypoint landed 4 m off -> `explore_and_map` converts the waypoints with
  the robot's spawn pose and its first pose in the map, then moves the saved map into the world frame.

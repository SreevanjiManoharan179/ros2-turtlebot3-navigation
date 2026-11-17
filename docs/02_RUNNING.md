# 2. Running the project

## 2.1 The demo menu (recommended)

```bash
cd ros2-turtlebot3-navigation      # the project folder
bash demo.sh
```

```
==============================================================
  Indoor Point-to-Point Autonomous Navigation - DEMO
  TurtleBot3 burger | ROS 2 humble | Gazebo Classic 11 | Nav2
==============================================================
  1) Room-to-room tour      (robot visits every room, point to point)
  2) Go to a room           (choose the destination yourself)
  3) Obstacle challenge     (an unmapped obstacle blocks the corridor)
  4) Full evaluation        (test table T1-T8 + report + figures, ~10 min)
  5) Build the map (SLAM)   (robot explores every room and saves a map)
  6) Drive it yourself      (keyboard, the map is built while you drive)
  7) Show latest results    (summary table + figures)
  8) Stop everything
  0) Exit
Choose:
```

Options 1-4 start Gazebo + Nav2 + RViz the first time (about 30-60 s), place RViz on the
left and Gazebo on the right, make the Gazebo camera follow the robot, and then run the
mission **in the same terminal**, which prints live progress and a results table. The
simulation keeps running between options, so the second run starts immediately.
`0) Exit` stops everything.

The same actions without the menu:

| Command | Does |
|---|---|
| `bash demo.sh tour` | room-to-room tour (Reception -> Lab -> Office -> Meeting Room -> Storage -> Kitchen -> Reception) |
| `bash demo.sh goto kitchen` | one point-to-point goal; a name from the table below or `x,y[,yaw_deg]` |
| `bash demo.sh obstacle` | an orange block that is **not in the map** appears in the corridor; the robot detects it with the LiDAR and re-plans through another room |
| `bash demo.sh eval` | the test table T1-T8 with metrics, report and plots in `results/` |
| `bash demo.sh map` | autonomous SLAM: the robot explores all rooms, saves a new map in `maps/`, anchors it to the floor plan and prints its accuracy |
| `bash demo.sh drive` | teleop (`w a s d x`) while SLAM builds a map |
| `bash demo.sh results` | prints the latest `summary.md` and opens the figures |
| `bash demo.sh up` | only start the simulation + navigation (then use RViz or `go_to` yourself) |
| `bash demo.sh stop` | stop everything |

Environment switches: `TB3_GUI=false` (no Gazebo window, faster), `TB3_RVIZ=false`,
`TB3_SIMULATOR=classic|gz` (force a simulator), `TURTLEBOT3_MODEL=waffle` (other robot).

## 2.2 Named locations (goals)

| Name | x [m] | y [m] | yaw [deg] | Room |
|---|---|---|---|---|
| reception (start) | -3.80 | -1.80 | 0 | Reception |
| lab | -3.00 | 1.60 | 90 | Lab |
| office | 0.60 | 2.30 | 90 | Office |
| meeting_room | 2.80 | 2.50 | 0 | Meeting Room |
| storage | 3.90 | -2.68 | 180 | Storage (between the shelves) |
| kitchen | -0.65 | -2.70 | 180 | Kitchen |
| corridor_east | 5.20 | 0.00 | 0 | Corridor |
| corridor_west | -5.30 | 0.00 | 180 | Corridor |

Coordinates are in the `map` frame, which equals the Gazebo world frame.

## 2.3 Step by step (what the menu does underneath)

Every terminal first needs the environment:

```bash
source /opt/ros/humble/setup.bash          # or jazzy
source ~/tb3_p2p_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
```

| Step | Command |
|---|---|
| 1. Simulation | `ros2 launch tb3_point_to_point simulation.launch.py` (add `gui:=false` for headless) |
| 2a. Build a map (teleop) | `ros2 launch tb3_point_to_point slam.launch.py` + `ros2 run turtlebot3_teleop teleop_keyboard` |
| 2b. Build a map (autonomous) | `ros2 launch tb3_point_to_point slam.launch.py explore:=true` + `ros2 run tb3_point_to_point explore_and_map --save ~/maps/my_map` (add `--reference <floor-plan>.yaml` to anchor and grade the map) |
| 3. Save a map | `ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --occ 0.65 --free 0.196 --ros-args -p use_sim_time:=true` |
| 4. Navigation | `ros2 launch tb3_point_to_point navigation.launch.py` (`map:=...` for another map, `rviz_view:=overview` for a top-down view) |
| 5. One goal (metrics) | `ros2 run tb3_point_to_point go_to kitchen` |
| 5'. One goal (minimal client) | `ros2 run tb3_point_to_point send_goal --ros-args -p x:=-3.0 -p y:=1.6 -p yaw:=1.5708 -p use_sim_time:=true` |
| 6. Missions | `ros2 run tb3_point_to_point mission_runner --scenario tour` (`evaluation`, `obstacle`) |
| 7. Report | `ros2 run tb3_point_to_point analyze_results results/<run folder>` |

## 2.4 Using RViz

* **Follow view** (default): a 3D camera behind the robot. Room names float above each room,
  the green line is the planned path, the coloured line is where the robot actually drove
  (one colour per goal), and the text above the robot shows the destination, distance left,
  speed and time.
* Switch to the **top-down view**: RViz *Views* panel -> *Whole floor (top-down)*, or start
  with `rviz_view:=overview`.
* **2D Pose Estimate**: fix the robot's position if localisation is lost.
* **Nav2 Goal**: click-and-drag a goal anywhere on the map (the classic manual way).

## 2.5 Changing things

| Want to... | Edit | Then |
|---|---|---|
| add / move a goal | `ros2_ws/src/tb3_point_to_point/config/locations.yaml` (or the `LOCATIONS` table in `tools/generate_indoor_office.py`) | `bash scripts/build.sh` |
| change the test table / tour | `config/scenarios.yaml` | `bash scripts/build.sh` |
| change the building / furniture | `tools/generate_indoor_office.py`, run `python3 tools/generate_indoor_office.py --png docs/images/floor_plan.png` | rebuild, then `bash demo.sh map` to make a new map |
| tune navigation (speed, tolerances, inflation) | `config/nav2_params_humble.yaml` (Jazzy: `nav2_params_jazzy.yaml`) | `bash scripts/build.sh` |

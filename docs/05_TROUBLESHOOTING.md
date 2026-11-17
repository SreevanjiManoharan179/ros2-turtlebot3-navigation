# 5. Troubleshooting

Logs of everything started by `demo.sh` are in **`logs/`** (`sim.log`, `nav.log`, `slam.log`).
First aid for almost everything: `bash scripts/stop.sh`, then try again.

| Symptom | Cause | Fix |
|---|---|---|
| `[spawn_entity]: Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?` | (1) an old `gzserver` is still running, (2) Gazebo hung downloading models from the internet, (3) Gazebo crashed (graphics), (4) terminal is root | `bash scripts/stop.sh` (kills stale gzserver/gzclient). The scripts already set `GAZEBO_MODEL_DATABASE_URI=""` (no downloads) and source `/usr/share/gazebo/setup.sh`. Never run as root. VM: `export LIBGL_ALWAYS_SOFTWARE=1`. |
| `ERROR: Do not run ROS 2 / Gazebo as root` | terminal opened with `sudo -s` / `su` | open a normal terminal. If files became root-owned: `sudo chown -R $USER:$USER ~/.ros ~/.gazebo ~/tb3_p2p_ws "<project folder>"` |
| `~/.bashrc still sources ROS 1 Noetic` | old ROS 1 line | remove/comment the line (setup.sh does it with a backup), open a new terminal, `bash scripts/build.sh` |
| `The workspace is not built yet` | first run | `bash scripts/build.sh` |
| `Package 'tb3_point_to_point' not found` | built under another shell / old build | `bash scripts/build.sh`; to rebuild from scratch: `rm -rf ~/tb3_p2p_ws` first |
| Gazebo window black / crashes, `GLXBadContext`, very low FPS | GPU driver / VM | NVIDIA: install the driver (`sudo ubuntu-drivers install`), use an Xorg session. VM/WSL2: `export LIBGL_ALWAYS_SOFTWARE=1` |
| `Nav2 did not become active` | Gazebo very slow, or a stale process on the same `ROS_DOMAIN_ID` | `bash scripts/stop.sh`; close other ROS programs; the project uses `ROS_DOMAIN_ID=30` |
| Robot does not move / goal rejected | AMCL has no pose, or goal is inside an obstacle | `go_to` checks the goal first and prints why it is invalid. In RViz use **2D Pose Estimate** to correct the pose, or `ros2 run tb3_point_to_point go_to <room> --init reception` |
| Robot spins, then "recovery" messages | LiDAR sees an obstacle that is not in the map, or the robot is mislocalised | normal behaviour for a short time (Nav2 recoveries). If it keeps failing: give a new 2D Pose Estimate, or clear costmaps from the Nav2 panel |
| Laser scan does not line up with the map in RViz | mislocalisation after a collision / teleport | 2D Pose Estimate at the true position |
| `Package 'gazebo_ros' not found` on Ubuntu 24.04 | Gazebo Classic does not exist for Jazzy | expected - Jazzy uses Gazebo Sim (`ros_gz`); `bash scripts/setup.sh` installs it |
| Windows are not placed side by side | `wmctrl` missing, or a Wayland session (Ubuntu 24.04 default) where apps may not move other windows | `sudo apt install wmctrl x11-utils`; on Wayland choose **Ubuntu on Xorg** on the login screen (gear icon) - or arrange the windows by hand, nothing else depends on it |
| Gazebo Sim / RViz window crashes or stays black on Ubuntu 24.04 | Qt apps on Wayland | `demo.sh` already sets `QT_QPA_PLATFORM=xcb` on Wayland; if you start things by hand, `export QT_QPA_PLATFORM=xcb` first, or log in with **Ubuntu on Xorg** |
| RViz shows only the 3D view, no Displays panel | the demo view (`rviz/navigation.rviz`) hides the side panels on purpose | menu **Panels -> Displays** (and **Views**) brings them back |
| `apt update` errors about `packages.ros.org` (`EXPKEYSIG`, `NO_PUBKEY`) | ROS signing key from before June 2025 | `bash scripts/setup.sh` removes the old key/list and installs `ros2-apt-source` |
| Map saved by `map_saver_cli` shows unknown areas as free when reloaded | Nav2's default `free_thresh` 0.25 reads grey (205) as free | save with `--free 0.196` (the scripts do), or edit `free_thresh: 0.196` in the YAML |

## Useful commands

```bash
ros2 topic list                      # is the simulation publishing? (/scan, /odom, /clock, /tf)
ros2 topic hz /scan                  # LiDAR running? (~5 Hz)
ros2 lifecycle get /bt_navigator     # Nav2 active?
ros2 run tf2_ros tf2_echo map base_footprint   # where AMCL thinks the robot is
ros2 run tb3_point_to_point go_to --list       # named goals
```

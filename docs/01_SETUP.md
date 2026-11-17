# 1. Setup (one time)

| Your Ubuntu | ROS 2 | Simulator | Status |
|---|---|---|---|
| 22.04 Jammy | Humble | Gazebo Classic 11 (`gazebo_ros`) | main target, fully tested |
| 24.04 Noble | Jazzy | Gazebo Sim / Harmonic (`ros_gz`) | supported, tested (test table, tour, obstacle, mapping, GUI) |

Use a **normal terminal as your own user** (not `sudo -s`, not `su`).

## 1.1 Install everything

```bash
git clone https://github.com/SreevanjiManoharan179/ros2-turtlebot3-navigation.git
cd ros2-turtlebot3-navigation
bash scripts/setup.sh           # asks for your sudo password once
```

`scripts/setup.sh` does the following (you can also type these yourself):

| Step | What | Why |
|---|---|---|
| 1 | `apt` update, `software-properties-common`, `curl`, universe repo | prerequisites |
| 2 | installs the **`ros2-apt-source`** package if the ROS repo is missing or its key is broken | official 2025+ method; replaces the old `ros.key` + `ros2.list` setup whose key expired on 1 June 2025 |
| 3 | `ros-<distro>-desktop navigation2 nav2-bringup nav2-simple-commander slam-toolbox turtlebot3 turtlebot3-msgs turtlebot3-simulations teleop-twist-keyboard` | ROS 2, Nav2, SLAM, TurtleBot3 (all from apt - no source builds of ROBOTIS packages) |
| 4 | Humble: `ros-humble-gazebo-ros-pkgs` / Jazzy: `ros-jazzy-ros-gz` | the simulator bridge |
| 5 | `python3-colcon-common-extensions python3-yaml python3-numpy python3-matplotlib rsync wmctrl x11-utils mesa-utils` | build tool, analysis plots, window layout |
| 6 | comments out a `source /opt/ros/noetic/setup.bash` line in `~/.bashrc` (backup `~/.bashrc.bak_tb3`) | ROS 1 + ROS 2 in one shell breaks everything |

## 1.2 Build

```bash
bash scripts/build.sh
```

The package is built in place in `ros2_ws/`. Run `build.sh` again after editing any file
under `ros2_ws/src`.

If the project folder path contains spaces or quotes (for example `My Robot's Project`),
colcon, the generated `setup.bash` files and Gazebo do not cope with it, so `build.sh` then
copies `ros2_ws/src` to **`~/tb3_p2p_ws`** and builds there automatically. Everything else
(results, maps, logs) stays inside the project folder.

## 1.3 Check

```bash
bash scripts/check.sh
```

Prints OK / WARN / FAIL for: Ubuntu version, not-root, no Noetic in `~/.bashrc`, ROS 2
found, every required package, workspace built, no stale Gazebo process, GPU / OpenGL
renderer, `wmctrl`.

## 1.4 NVIDIA laptops (Intel + RTX, e.g. Lenovo LOQ)

Install the proprietary driver once (`sudo ubuntu-drivers install`, reboot). The scripts set
`__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` automatically when
`nvidia-smi` works, so Gazebo and RViz render on the RTX GPU. Prefer an **"Ubuntu on Xorg"**
login session for Gazebo (gear icon on the login screen).

## 1.5 Virtual machines / WSL2

Works, but slower. If the Gazebo window is black or crashes, run
`export LIBGL_ALWAYS_SOFTWARE=1` before `bash demo.sh` (software rendering), and/or
disable 3D acceleration in VirtualBox/VMware. Expect a real-time factor below 1 - the
navigation still works because every node uses simulation time.

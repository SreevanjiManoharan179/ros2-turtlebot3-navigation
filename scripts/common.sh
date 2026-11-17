#!/usr/bin/env bash
# Shared settings and helpers for every script of this project.
# Source it (". scripts/common.sh"); do not run it directly.

# ----------------------------------------------------------------- locations
# The project folder may have any name, even with spaces or an apostrophe
# (e.g. "My Robot's Project").  ROS 2 build tools and Gazebo do not cope well with such
# paths, so in that case the workspace is built in a plain path in your home folder
# and everything else (results, maps, logs) stays in the project folder.
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG=tb3_point_to_point
SRC_DIR="$PROJECT_ROOT/ros2_ws/src"
if [[ "$PROJECT_ROOT" =~ ^[A-Za-z0-9_./-]+$ ]]; then
  WS_DIR="${TB3_WS:-$PROJECT_ROOT/ros2_ws}"
else
  WS_DIR="${TB3_WS:-$HOME/tb3_p2p_ws}"
fi
RESULTS_DIR="$PROJECT_ROOT/results"
MAPS_DIR="$PROJECT_ROOT/maps"
LOG_DIR="$PROJECT_ROOT/logs"
RUN_DIR="${XDG_RUNTIME_DIR:-/tmp}/tb3_p2p_${USER:-user}"
mkdir -p "$RESULTS_DIR" "$MAPS_DIR" "$LOG_DIR" "$RUN_DIR" 2>/dev/null

# ------------------------------------------------------------ environment
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export GAZEBO_MODEL_DATABASE_URI=""        # never wait for online model downloads (classic hang)
export GAZEBO_SUPPRESS_EOL_WARNING=1
export PYTHONUNBUFFERED=1

# ----------------------------------------------------------------- output
if [ -t 1 ]; then
  C_RESET=$'\033[0m'; C_BOLD=$'\033[1m'; C_DIM=$'\033[2m'; C_RED=$'\033[31m'
  C_GREEN=$'\033[32m'; C_YELLOW=$'\033[33m'; C_CYAN=$'\033[36m'
else
  C_RESET=; C_BOLD=; C_DIM=; C_RED=; C_GREEN=; C_YELLOW=; C_CYAN=
fi
info() { printf '%s\n' "${C_CYAN}$*${C_RESET}"; }
ok()   { printf '%s\n' "${C_GREEN}$*${C_RESET}"; }
warn() { printf '%s\n' "${C_YELLOW}WARNING: $*${C_RESET}" >&2; }
die()  { printf '%s\n' "${C_RED}ERROR: $*${C_RESET}" >&2; exit 1; }

# ------------------------------------------------------------- safety checks
refuse_root() {
  if [ "$(id -u)" -eq 0 ] && [ -z "${TB3_ALLOW_ROOT:-}" ]; then
    die "Do not run ROS 2 / Gazebo as root (sudo -s, su). Open a normal terminal as your own user.
       Running as root uses /root as HOME, breaks the Gazebo window and leaves root-owned files behind."
  fi
}

warn_noetic() {
  if grep -qs '^[^#]*/opt/ros/noetic' "$HOME/.bashrc"; then
    warn "~/.bashrc still sources ROS 1 Noetic. That mixes ROS 1 and ROS 2 and breaks this project.
         Fix: open ~/.bashrc, put a # in front of the 'source /opt/ros/noetic/setup.bash' line, open a new terminal."
  fi
}

# ------------------------------------------------------------- ROS sourcing
detect_ros_setup() {
  ROS_SETUP=""
  if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
  else
    for d in humble jazzy kilted rolling; do
      if [ -f "/opt/ros/$d/setup.bash" ]; then ROS_SETUP="/opt/ros/$d/setup.bash"; break; fi
    done
  fi
  [ -n "$ROS_SETUP" ] || { [ -n "${ROS_DISTRO:-}" ] && command -v ros2 >/dev/null 2>&1; }
}

source_ros() {
  detect_ros_setup || die "ROS 2 was not found in /opt/ros. Run: bash scripts/setup.sh"
  set +u
  # shellcheck disable=SC1090
  [ -n "$ROS_SETUP" ] && . "$ROS_SETUP"
  # Gazebo Classic resource paths (Humble)
  # shellcheck disable=SC1091
  [ -f /usr/share/gazebo/setup.sh ] && . /usr/share/gazebo/setup.sh
  # shellcheck disable=SC1091
  [ -n "${CONDA_PREFIX:-}" ] && [ -f "$CONDA_PREFIX/share/gazebo/setup.sh" ] && . "$CONDA_PREFIX/share/gazebo/setup.sh"
  # Wayland desktops (Ubuntu 24.04 default): run the Qt apps (Gazebo GUI, RViz) through XWayland,
  # which they are tested with; window placement by this script also needs X11 windows
  if [ "${XDG_SESSION_TYPE:-}" = "wayland" ] && [ -z "${QT_QPA_PLATFORM:-}" ]; then
    export QT_QPA_PLATFORM=xcb
  fi
  # NVIDIA laptops (Intel + RTX): render Gazebo/RViz on the NVIDIA GPU when the driver works
  if [ -z "${TB3_NO_PRIME:-}" ] && command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
  fi
  export RCUTILS_COLORIZED_OUTPUT=1
}

source_workspace() {
  source_ros
  [ -f "$WS_DIR/install/setup.bash" ] || die "The workspace is not built yet. Run: bash scripts/build.sh"
  set +u
  # shellcheck disable=SC1091
  . "$WS_DIR/install/setup.bash"
  ros2 pkg prefix "$PKG" >/dev/null 2>&1 || die "Package $PKG not found after sourcing $WS_DIR. Run: bash scripts/build.sh"
}

simulator_name() {
  case "${TB3_SIMULATOR:-}" in classic) echo "Gazebo Classic 11"; return ;; gz) echo "Gazebo Sim"; return ;; esac
  if [ "${ROS_DISTRO:-}" = humble ]; then echo "Gazebo Classic 11"; else echo "Gazebo Sim"; fi
}

# ------------------------------------------------- background process control
# start_bg NAME CMD...  - runs CMD in its own process group, output to logs/NAME.log
start_bg() {
  local name="$1"; shift
  setsid "$@" > "$LOG_DIR/$name.log" 2>&1 < /dev/null &
  echo $! > "$RUN_DIR/$name.pid"
}

is_running() {
  local f="$RUN_DIR/$1.pid"
  [ -f "$f" ] && kill -0 "$(cat "$f")" 2>/dev/null
}

stop_bg() {
  local f="$RUN_DIR/$1.pid" pid
  [ -f "$f" ] || return 0
  pid="$(cat "$f")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -TERM -- "-$pid" 2>/dev/null; sleep 0.5
    kill -KILL -- "-$pid" 2>/dev/null
  fi
  rm -f "$f"
}

stop_everything() {
  local n
  for n in mission teleop explore slam nav sim; do stop_bg "$n"; done
  # leftovers that block the next start (a stale gzserver is the #1 cause of
  # "Service /spawn_entity unavailable")
  pkill -f "$PKG/(mission_runner|go_to|explore_and_map|room_labels)" 2>/dev/null
  pkill -f "rviz2.*$PKG" 2>/dev/null
  killall -q -9 gzserver gzclient 2>/dev/null
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -f "parameter_bridge.*turtlebot3" 2>/dev/null
  sleep 1
}

# ------------------------------------------------------------- waiting helpers
wait_for() {   # wait_for SECONDS DESCRIPTION COMMAND...
  local secs="$1" what="$2"; shift 2
  local t=0
  printf '%s' "${C_DIM}  waiting for $what ${C_RESET}"
  while ! "$@" >/dev/null 2>&1; do
    sleep 2; t=$((t + 2)); printf '.'
    if [ "$t" -ge "$secs" ]; then printf '\n'; return 1; fi
  done
  printf ' %s\n' "${C_GREEN}ok${C_RESET}"
}

robot_spawned() { grep -qE "Successfully spawned|Entity creation successful|Requested creation of entity" "$LOG_DIR/sim.log" 2>/dev/null; }
nav2_active()   { ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q '^active'; }
odom_alive()    { timeout 8 ros2 topic echo /odom --once >/dev/null 2>&1; }

# ------------------------------------------------------------- window layout
# Like the demo video: RViz on the left half, Gazebo on the right half and, when the
# terminal window can be identified (xdotool), Gazebo top-right + terminal bottom-right.
# Best effort: needs wmctrl (+ xdotool, x11-utils) and an X11/XWayland session.
remember_terminal_window() {
  if [ -n "${DISPLAY:-}" ] && command -v xdotool >/dev/null 2>&1; then
    TB3_TERMINAL_WINDOW="$(xdotool getactivewindow 2>/dev/null || true)"
  fi
  export TB3_TERMINAL_WINDOW
}

arrange_windows() {
  [ -n "${DISPLAY:-}" ] || return 0
  command -v wmctrl >/dev/null 2>&1 || return 0
  command -v xdpyinfo >/dev/null 2>&1 || return 0
  local dim w h half top
  dim="$(xdpyinfo 2>/dev/null | awk '/dimensions:/{print $2; exit}')"
  w="${dim%x*}"; h="${dim#*x}"
  [ -n "$w" ] && [ -n "$h" ] || return 0
  half=$((w / 2)); top=$((h * 62 / 100))
  wmctrl -r "RViz" -b remove,maximized_vert,maximized_horz 2>/dev/null
  wmctrl -r "RViz" -e "0,0,0,$half,$h" 2>/dev/null
  wmctrl -r "Gazebo" -b remove,maximized_vert,maximized_horz 2>/dev/null
  if [ -n "${TB3_TERMINAL_WINDOW:-}" ]; then
    wmctrl -r "Gazebo" -e "0,$half,0,$half,$top" 2>/dev/null
    wmctrl -i -r "$TB3_TERMINAL_WINDOW" -b remove,maximized_vert,maximized_horz 2>/dev/null
    wmctrl -i -r "$TB3_TERMINAL_WINDOW" -e "0,$half,$top,$half,$((h - top))" 2>/dev/null
    wmctrl -i -a "$TB3_TERMINAL_WINDOW" 2>/dev/null
  else
    wmctrl -r "Gazebo" -e "0,$half,0,$half,$h" 2>/dev/null
  fi
}

follow_robot() {
  if [ "${ROS_DISTRO:-}" = humble ] && [ "${TB3_SIMULATOR:-classic}" != gz ]; then
    timeout 10 gz camera -c gzclient_camera -f "$TURTLEBOT3_MODEL" >/dev/null 2>&1
  else
    timeout 10 gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
      --timeout 3000 --req "data: \"$TURTLEBOT3_MODEL\"" >/dev/null 2>&1
    timeout 10 gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean \
      --timeout 3000 --req "x: -2.2, y: 0.0, z: 2.0" >/dev/null 2>&1
  fi
  return 0
}

latest_dir() {  # newest results folder matching a prefix
  ls -1dt "$RESULTS_DIR"/"$1"* 2>/dev/null | head -n 1
}

active_map() {
  if [ -f "$MAPS_DIR/ACTIVE_MAP" ] && [ -f "$(cat "$MAPS_DIR/ACTIVE_MAP")" ]; then
    cat "$MAPS_DIR/ACTIVE_MAP"
  else
    echo "$(ros2 pkg prefix "$PKG")/share/$PKG/maps/indoor_office.yaml"
  fi
}

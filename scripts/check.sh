#!/usr/bin/env bash
# Check that this computer is ready for the project (read-only, changes nothing).
#   bash scripts/check.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"

PASS=0; WARNS=0; FAILS=0
row()  { printf '  %-30s %s\n' "$1" "$2"; }
pass() { row "$1" "${C_GREEN}OK${C_RESET}    $2"; PASS=$((PASS + 1)); }
wrn()  { row "$1" "${C_YELLOW}WARN${C_RESET}  $2"; WARNS=$((WARNS + 1)); }
fail() { row "$1" "${C_RED}FAIL${C_RESET}  $2"; FAILS=$((FAILS + 1)); }

echo "${C_BOLD}System check - Indoor Point-to-Point Navigation (TurtleBot3)${C_RESET}"
# shellcheck disable=SC1091
. /etc/os-release 2>/dev/null
case "${VERSION_ID:-}" in
  22.04|24.04) pass "Ubuntu" "$PRETTY_NAME" ;;
  *) fail "Ubuntu" "${PRETTY_NAME:-unknown} (use 22.04 or 24.04)" ;;
esac
if [ "$(id -u)" -eq 0 ]; then fail "User" "running as root - use your normal user"; else pass "User" "$USER (not root)"; fi
if grep -qs '^[^#]*/opt/ros/noetic' "$HOME/.bashrc"; then wrn "~/.bashrc" "sources ROS 1 Noetic (remove that line)"; else pass "~/.bashrc" "no ROS 1 Noetic line"; fi

if detect_ros_setup; then
  set +u
  # shellcheck disable=SC1090
  [ -n "$ROS_SETUP" ] && . "$ROS_SETUP"
  pass "ROS 2" "$ROS_DISTRO ($(simulator_name))"
  for p in nav2_bringup nav2_simple_commander slam_toolbox turtlebot3_gazebo turtlebot3_description rviz2; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then pass "  package $p" "found"; else fail "  package $p" "missing - run bash scripts/setup.sh"; fi
  done
  if [ "$ROS_DISTRO" = humble ]; then sim_pkg=gazebo_ros; else sim_pkg=ros_gz_sim; fi
  if ros2 pkg prefix "$sim_pkg" >/dev/null 2>&1; then pass "  package $sim_pkg" "found"; else fail "  package $sim_pkg" "missing - run bash scripts/setup.sh"; fi
  if [ -f "$WS_DIR/install/setup.bash" ]; then pass "Workspace" "built in $WS_DIR"; else wrn "Workspace" "not built yet - run bash scripts/build.sh"; fi
else
  fail "ROS 2" "not found in /opt/ros - run bash scripts/setup.sh"
fi

if pgrep -x gzserver >/dev/null || pgrep -f "gz sim" >/dev/null; then
  wrn "Gazebo processes" "already running (bash scripts/stop.sh clears them)"
else
  pass "Gazebo processes" "none left over"
fi
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  pass "GPU" "$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n1) (PRIME offload enabled)"
else
  wrn "GPU" "no working NVIDIA driver - Gazebo will use the integrated/software renderer"
fi
if command -v glxinfo >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  row "OpenGL renderer" "$(glxinfo -B 2>/dev/null | awk -F': ' '/OpenGL renderer/{print $2}')"
fi
if command -v wmctrl >/dev/null 2>&1; then pass "wmctrl" "windows will be placed side by side"; else wrn "wmctrl" "not installed (optional: sudo apt install wmctrl)"; fi
row "Project folder" "$PROJECT_ROOT"
echo
echo "  ${C_GREEN}$PASS ok${C_RESET}, ${C_YELLOW}$WARNS warnings${C_RESET}, ${C_RED}$FAILS failures${C_RESET}"
[ "$FAILS" -eq 0 ]

#!/usr/bin/env bash
# =============================================================================
#  Indoor Point-to-Point Autonomous Navigation - TurtleBot3 + ROS 2 + Gazebo
#  Demo launcher.  Run from a normal (non-root) terminal:
#
#      bash demo.sh              interactive menu
#      bash demo.sh tour         room-to-room tour
#      bash demo.sh goto kitchen one point-to-point goal (name or x,y[,yaw_deg])
#      bash demo.sh obstacle     unmapped obstacle + re-planning
#      bash demo.sh eval         full test table + report
#      bash demo.sh map          robot builds a new map by itself (SLAM)
#      bash demo.sh drive        drive with the keyboard while mapping
#      bash demo.sh results      show the latest results
#      bash demo.sh up           only start Gazebo + Nav2 + RViz
#      bash demo.sh stop         stop everything
# =============================================================================
DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/common.sh
. "$DEMO_DIR/scripts/common.sh"
refuse_root
warn_noetic
source_workspace
remember_terminal_window

stamp() { date +%Y%m%d_%H%M%S; }

# ---------------------------------------------------------------- the stack
stack_up() {   # Gazebo + robot + Nav2 + RViz, unless they are already running
  local view="${1:-follow}"
  if is_running sim && is_running nav && nav2_active; then return 0; fi
  stop_everything
  info "Starting $(simulator_name): indoor office world + TurtleBot3 ${TURTLEBOT3_MODEL} ..."
  start_bg sim ros2 launch "$PKG" simulation.launch.py gui:="${TB3_GUI:-true}"
  if ! wait_for 240 "the robot to appear in Gazebo" robot_spawned; then
    tail -n 25 "$LOG_DIR/sim.log"
    die "Gazebo did not spawn the robot (full log: logs/sim.log). Run 'bash demo.sh stop' and try again."
  fi
  info "Starting Nav2 + RViz with map $(basename "$(active_map)") ..."
  start_bg nav ros2 launch "$PKG" navigation.launch.py map:="$(active_map)" \
    rviz_view:="$view" rviz:="${TB3_RVIZ:-true}"
  if ! wait_for 240 "Nav2 to become active" nav2_active; then
    tail -n 25 "$LOG_DIR/nav.log"
    die "Nav2 did not become active (full log: logs/nav.log)."
  fi
  sleep 4
  arrange_windows
  follow_robot
  ok "Simulation and navigation are running."
}

run_mission() {   # run_mission SCENARIO
  local out
  out="$RESULTS_DIR/${1}_$(stamp)"
  stack_up follow
  ros2 run "$PKG" mission_runner --scenario "$1" --out "$out"
  echo
  info "Report: $out/summary.md   Figures: $out/trajectories.png, $out/metrics.png"
}

do_goto() {
  stack_up follow
  local dest="$1"
  if [ -z "$dest" ]; then
    ros2 run "$PKG" go_to --list
    read -r -p "Destination (name, or x,y[,yaw_deg]): " dest
    [ -n "$dest" ] || return 0
  fi
  ros2 run "$PKG" go_to "$dest" --csv "$RESULTS_DIR/go_to_log.csv"
}

do_map() {
  local out
  out="$MAPS_DIR/indoor_office_slam_$(stamp)"
  stop_everything
  info "Starting $(simulator_name) + SLAM Toolbox + Nav2 (no prior map) ..."
  start_bg sim ros2 launch "$PKG" simulation.launch.py gui:="${TB3_GUI:-true}"
  wait_for 240 "the robot to appear in Gazebo" robot_spawned || die "Gazebo did not start (logs/sim.log)"
  start_bg slam ros2 launch "$PKG" slam.launch.py explore:=true rviz_view:=overview
  wait_for 240 "Nav2 to become active" nav2_active || die "Nav2 did not start (logs/slam.log)"
  sleep 3; arrange_windows; follow_robot
  ros2 run "$PKG" explore_and_map --save "$out" \
      --reference "$(ros2 pkg prefix "$PKG")/share/$PKG/maps/indoor_office_ground_truth.yaml" \
      || warn "mapping finished with problems"
  if [ -f "$out.yaml" ]; then
    ok "New map: $out.yaml"
    read -r -p "Use this map for navigation from now on? [y/N] " a
    if [[ "$a" =~ ^[Yy] ]]; then echo "$out.yaml" > "$MAPS_DIR/ACTIVE_MAP"; ok "Active map set."; fi
  fi
  stop_everything
}

do_drive() {
  stop_everything
  start_bg sim ros2 launch "$PKG" simulation.launch.py gui:="${TB3_GUI:-true}"
  wait_for 240 "the robot to appear in Gazebo" robot_spawned || die "Gazebo did not start (logs/sim.log)"
  start_bg slam ros2 launch "$PKG" slam.launch.py rviz_view:=overview
  sleep 8; arrange_windows; follow_robot
  info "Keyboard control: w/x = faster/slower forward, a/d = turn, s or space = stop, Ctrl+C = finish"
  ros2 run turtlebot3_teleop teleop_keyboard
  read -r -p "Save the map you just built? [y/N] " a
  if [[ "$a" =~ ^[Yy] ]]; then
    local out
    out="$MAPS_DIR/manual_map_$(stamp)"
    ros2 run nav2_map_server map_saver_cli -f "$out" --occ 0.65 --free 0.196 --ros-args -p use_sim_time:=true \
      && ok "Saved $out.yaml (note: its frame starts where the robot started)"
  fi
  stop_everything
}

do_results() {
  local d
  d="$(ls -1dt "$RESULTS_DIR"/*/ 2>/dev/null | head -n 1)"
  [ -n "$d" ] || { warn "No results yet - run the tour or the evaluation first."; return 0; }
  d="${d%/}"
  info "Latest results: $d"
  if [ -f "$d/summary.md" ]; then cat "$d/summary.md"; else ros2 run "$PKG" analyze_results "$d"; fi
  if command -v xdg-open >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    for f in "$d/trajectories.png" "$d/metrics.png"; do [ -f "$f" ] && xdg-open "$f" >/dev/null 2>&1 & done
  fi
}

menu() {
  while true; do
    echo
    echo "${C_BOLD}==============================================================${C_RESET}"
    echo "${C_BOLD}  Indoor Point-to-Point Autonomous Navigation - DEMO${C_RESET}"
    echo "  TurtleBot3 ${TURTLEBOT3_MODEL} | ROS 2 ${ROS_DISTRO} | $(simulator_name) | Nav2"
    echo "${C_BOLD}==============================================================${C_RESET}"
    echo "  1) Room-to-room tour      (robot visits every room, point to point)"
    echo "  2) Go to a room           (choose the destination yourself)"
    echo "  3) Obstacle challenge     (an unmapped obstacle blocks the corridor)"
    echo "  4) Full evaluation        (test table T1-T8 + report + figures, ~10 min)"
    echo "  5) Build the map (SLAM)   (robot explores every room and saves a map)"
    echo "  6) Drive it yourself      (keyboard, the map is built while you drive)"
    echo "  7) Show latest results    (summary table + figures)"
    echo "  8) Stop everything"
    echo "  0) Exit"
    read -r -p "Choose: " choice || exit 0
    trap 'echo; warn "interrupted - back to the menu"' INT
    case "$choice" in
      1) run_mission tour ;;
      2) do_goto "" ;;
      3) run_mission obstacle ;;
      4) run_mission evaluation ;;
      5) do_map ;;
      6) do_drive ;;
      7) do_results ;;
      8) stop_everything; ok "Stopped." ;;
      0) stop_everything; exit 0 ;;
      *) warn "unknown choice: $choice" ;;
    esac
    trap - INT
  done
}

case "${1:-menu}" in
  menu)        menu ;;
  up)          stack_up "${2:-follow}" ;;
  tour)        run_mission tour ;;
  goto)        do_goto "${2:-}" ;;
  obstacle)    run_mission obstacle ;;
  eval|evaluation) run_mission evaluation ;;
  map)         do_map ;;
  drive)       do_drive ;;
  results)     do_results ;;
  stop)        stop_everything; ok "Stopped." ;;
  *)           sed -n '2,19p' "$0"; exit 1 ;;
esac

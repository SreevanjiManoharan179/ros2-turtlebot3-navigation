#!/usr/bin/env bash
# Build the ROS 2 package of this project (tb3_point_to_point).
#
#   bash scripts/build.sh
#
# If the project folder name contains spaces or quotes (e.g. "My Robot's Project"), the
# sources are mirrored to ~/tb3_p2p_ws and built there, because colcon and Gazebo do not
# handle such paths.  Run this again after you edit anything under ros2_ws/src.
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
refuse_root
warn_noetic
source_ros
command -v colcon >/dev/null 2>&1 || die "colcon not found. Run: bash scripts/setup.sh"

info "Project   : $PROJECT_ROOT"
info "ROS 2     : ${ROS_DISTRO}   (simulator: $(simulator_name))"
info "Workspace : $WS_DIR"
if [ "$WS_DIR" != "$PROJECT_ROOT/ros2_ws" ]; then
  info "            (the project path contains spaces/quotes, so the package is built in a plain path)"
  mkdir -p "$WS_DIR/src" || die "cannot create $WS_DIR"
  if command -v rsync >/dev/null 2>&1; then
    rsync -a --delete "$SRC_DIR/" "$WS_DIR/src/"
  else
    rm -rf "${WS_DIR:?}/src" && mkdir -p "$WS_DIR/src" && cp -a "$SRC_DIR/." "$WS_DIR/src/"
  fi
fi

cd "$WS_DIR" || die "cannot enter $WS_DIR"
colcon build --packages-select "$PKG" "$@" || die "colcon build failed - read the messages above"
ok "Build finished. Start the demo with:  bash demo.sh"

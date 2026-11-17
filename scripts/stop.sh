#!/usr/bin/env bash
# Stop Gazebo, Nav2, RViz and every node started by this project.
#   bash scripts/stop.sh
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
stop_everything
ok "Everything stopped."

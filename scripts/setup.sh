#!/usr/bin/env bash
# One-time installation of everything this project needs (asks for your sudo password).
#
#   bash scripts/setup.sh
#
#   Ubuntu 22.04  ->  ROS 2 Humble + Gazebo Classic 11 + Nav2 + SLAM Toolbox + TurtleBot3
#   Ubuntu 24.04  ->  ROS 2 Jazzy  + Gazebo Sim (Harmonic) + Nav2 + SLAM Toolbox + TurtleBot3
#
# Safe to run again: already-installed packages are skipped.
set -e
. "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
refuse_root

# shellcheck disable=SC1091
. /etc/os-release
case "${VERSION_ID:-}" in
  22.04) DISTRO=humble ;;
  24.04) DISTRO=jazzy ;;
  *) die "Ubuntu ${VERSION_ID:-?} is not supported. Use Ubuntu 22.04 (Humble) or 24.04 (Jazzy)." ;;
esac
info "Ubuntu $VERSION_ID detected -> installing ROS 2 $DISTRO"

sudo apt-get update
sudo apt-get install -y software-properties-common curl locales
sudo locale-gen en_US en_US.UTF-8 >/dev/null
sudo add-apt-repository -y universe

# ---- ROS 2 apt repository --------------------------------------------------
# Current official method: the ros2-apt-source package (key + source list in one
# package; future key rotations arrive with normal updates).  It also repairs the
# old manual setup whose signing key expired on 1 June 2025.
repo_ok()    { grep -rqs "packages.ros.org/ros2" /etc/apt/sources.list /etc/apt/sources.list.d/; }
key_broken() { sudo apt-get update 2>&1 | grep "packages.ros.org" | grep -qE "EXPKEYSIG|NO_PUBKEY|not signed|invalid"; }
if ! dpkg -s ros2-apt-source >/dev/null 2>&1 && { ! repo_ok || key_broken; }; then
  info "Setting up the ROS 2 apt repository (ros2-apt-source)"
  sudo rm -f /etc/apt/sources.list.d/ros2.list /usr/share/keyrings/ros-archive-keyring.gpg
  VER="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
         | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  [ -n "$VER" ] || die "Could not look up the ros-apt-source release (check the internet connection)."
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VER}/ros2-apt-source_${VER}.${UBUNTU_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
fi

# ---- packages ------------------------------------------------------------------
P="ros-$DISTRO"
PKGS=(
  "$P-desktop" "$P-navigation2" "$P-nav2-bringup" "$P-nav2-simple-commander"
  "$P-slam-toolbox" "$P-turtlebot3" "$P-turtlebot3-msgs" "$P-turtlebot3-simulations"
  "$P-teleop-twist-keyboard"
  python3-colcon-common-extensions python3-yaml python3-numpy python3-matplotlib
  rsync wmctrl xdotool x11-utils mesa-utils
)
if [ "$DISTRO" = humble ]; then PKGS+=("$P-gazebo-ros-pkgs"); else PKGS+=("$P-ros-gz"); fi
info "Installing: ${PKGS[*]}"
sudo apt-get install -y "${PKGS[@]}"

# ---- ~/.bashrc hygiene -------------------------------------------------------
if grep -qs '^[^#]*/opt/ros/noetic' "$HOME/.bashrc"; then
  cp "$HOME/.bashrc" "$HOME/.bashrc.bak_tb3"
  sed -i 's|^\([^#]*/opt/ros/noetic.*\)$|# \1   # disabled by tb3 project setup (ROS 1 + ROS 2 do not mix)|' "$HOME/.bashrc"
  warn "Commented out the ROS 1 Noetic line in ~/.bashrc (backup: ~/.bashrc.bak_tb3). Open a NEW terminal."
fi

ok "All dependencies are installed for ROS 2 $DISTRO."
ok "Next:  bash scripts/build.sh   then   bash demo.sh"

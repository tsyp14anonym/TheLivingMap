#!/usr/bin/env bash
# Installs ROS 2 Jazzy (if missing), Gazebo Harmonic (the 'gz' command), the ROS-Gazebo bridge and RViz on Ubuntu 24.04.
# Needs internet and sudo.   usage: bash install_ros_gz.sh
set -e
. /etc/os-release
if [ "$VERSION_ID" != "24.04" ]; then echo "This script is for Ubuntu 24.04 (ROS 2 Jazzy). You have $VERSION_ID. For 22.04 use ROS 2 Humble + Gazebo Fortress/Harmonic (see docs.ros.org)."; exit 1; fi

if [ ! -d /opt/ros/jazzy ]; then
  echo ">>> Installing ROS 2 Jazzy (official method from docs.ros.org)"
  sudo apt install -y software-properties-common curl
  sudo add-apt-repository -y universe
  sudo apt update
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update && sudo apt upgrade -y
  sudo apt install -y ros-jazzy-desktop ros-dev-tools
fi

echo ">>> Adding the official Gazebo repository and installing Gazebo Harmonic (gz command)"
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic ros-jazzy-ros-gz ros-jazzy-rviz2 python3-colcon-common-extensions mesa-utils
sudo apt install -y python3-gz-transport13 python3-gz-msgs10 || echo "(optional gz python bindings not available: Gazebo motion will use the slower CLI mode)"

grep -q "opt/ros/jazzy/setup.bash" ~/.bashrc || echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo; echo "DONE. Close this terminal, open a new one and run:  gz sim --version   then   bash doctor.sh"

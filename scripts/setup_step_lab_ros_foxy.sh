#!/usr/bin/env bash
set -euo pipefail

# Run on 172.16.0.250 (Ubuntu 20.04) when root/sudo package installation is desired.
# This script is intentionally explicit and does not run automatically from Codex.

sudo apt update
sudo apt install -y curl gnupg2 lsb-release locales software-properties-common
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt install -y ros-foxy-desktop python3-colcon-common-extensions python3-rosdep python3-pip
sudo rosdep init || true
rosdep update

echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc

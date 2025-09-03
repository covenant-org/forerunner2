#!/bin/bash
set -e

# update system
sudo apt-get update

# install dependencies
sudo apt-get install -y libpcap-dev libpcl-dev libyaml-cpp-dev

# reclaim px4 permissions
sudo usermod -aG px4 $USER
git config --global --add safe.directory /usr/local/share/px4
sudo chown $USER:$USER -R /usr/local/share/px4
cd /usr/local/share/px4

# update submodules of px4
git stash
bash ./Tools/setup/ubuntu.sh
git submodule update --init --recursive
git pull

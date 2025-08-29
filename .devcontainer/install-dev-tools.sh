# update system
sudo apt-get update

# reclaim px4 permissions
git config --global --add safe.directory /tools/PX4-Autopilot
sudo chown $USER:$USER -R /tools/PX4-Autopilot

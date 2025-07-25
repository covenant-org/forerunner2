# update system
sudo apt-get update

# reclaim px4 permissions
sudo usermod -aG px4 $USER
git config --global --add safe.directory /usr/local/share/px4
sudo chown $USER:$USER -R /usr/local/share/px4
cd /usr/local/share/px4

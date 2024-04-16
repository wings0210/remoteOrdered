#! /bin/bash
gnome-terminal --tab -- bash -c "sudo apt-get install ros-noetic-rosauth"
sleep 1s
cd ../hmi_middleware
sudo rosdep install --from-paths src --ignore-src -r -y

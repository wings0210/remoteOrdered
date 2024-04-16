#! /bin/bash
cd ../hmi_middleware

gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch rosbridge_server rosbridge_websocket.launch"

sleep 3s

gnome-terminal --tab -- bash -c "source devel/setup.bash; roslaunch ros_mutual ros_mutual.launch"

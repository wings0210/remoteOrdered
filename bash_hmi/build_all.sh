#! /bin/bash
cd ../hmi_middleware
rm -rf build/ devel/ .catkin_workspace
sleep 1s
catkin_make --only-pkg-with-deps rosbridge_suite
sleep 5s
catkin_make --only-pkg-with-deps ros_mutual
sleep 5s

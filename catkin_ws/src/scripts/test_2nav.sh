#!/bin/sh

xterm  -e  " roslaunch my_robot world.launch" &
sleep 5
xterm  -e  " roslaunch my_robot amcl.launch " &
sleep 5
xterm  -e  " rosrun rviz rviz -d $(pwd)/src/rvizConfig/final.rviz"


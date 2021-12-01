#!/bin/sh

xterm  -e  " roslaunch my_robot world.launch" &
sleep 5
xterm  -e  " roslaunch my_robot mapping.launch " &
sleep 5
xterm  -e  " rosrun rviz rviz -d $(pwd)/src/rvizConfig/final.rviz" &
sleep 5
xterm -e " roslaunch my_robot teleop.launch "

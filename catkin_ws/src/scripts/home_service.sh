#!/bin/sh
xterm  -e  " roslaunch my_robot world.launch " &
sleep 5
xterm  -e  " roslaunch my_robot amcl.launch map_file:=$(pwd)/src/map/map.yaml " &
sleep 5
xterm  -e  " rosrun rviz rviz -d $(pwd)/src/rvizConfig/final.rviz " &
sleep 5
xterm -e   " rosrun add_markers add_markers " &
sleep 5
xterm  -e  " rosrun pick_objects pick_objects "

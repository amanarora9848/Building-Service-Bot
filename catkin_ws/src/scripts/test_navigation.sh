#!/bin/sh

# Currently, I am using my own robot and world to perform navigation.

xterm  -e  " roslaunch my_robot world.launch " &
sleep 5
xterm  -e  " roslaunch my_robot amcl.launch map_file:=$(pwd)/src/map/map.yaml " &
sleep 5
xterm  -e  " rosrun rviz rviz -d $(pwd)/src/rvizConfig/final.rviz "
# sleep 5
# xterm  -e  " roslaunch my_robot teleop.launch "


# Although, by uncommenting the below lines, we can also do the turtlebot implementation.

# export TURTLEBOT_GAZEBO_WORLD_FILE=$(pwd)/src/my_robot/world/AmanWorld.world
# xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
# sleep 5
# xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(rospack find my_robot)/maps/map.yaml " &
# sleep 5
# xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " & 
# sleep 5
# xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch " 

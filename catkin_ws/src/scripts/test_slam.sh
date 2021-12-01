#!/bin/sh

# Currently, I am using my own robot and world for testing SLAM.

xterm  -e  " roslaunch my_robot world.launch " &
sleep 5
xterm  -e  " rosrun rviz rviz -d $(pwd)/src/rvizConfig/final.rviz " &
sleep 5
xterm  -e  " roslaunch my_robot mapping.launch " &
sleep 5
xterm  -e  " rosrun teleop_twist_keyboard teleop_twist_keyboard.py "

# Although, uncommenting these, we can also run the turtlebot implementation.

# export TURTLEBOT_GAZEBO_WORLD_FILE=$(pwd)/src/my_robot/world/AmanWorld.world
# xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch " &
# sleep 5
# xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch " &
# sleep 5
# xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch " & 
# sleep 5
# xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch " 

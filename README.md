# Building-Service-Bot


##### This is a mini-project created to demonstrate a mobile service robot, capable of mapping its environment and autonomously navigating through it. It is capable of moving towards a certain goal, pick up an object, and drop it off at a certain other location.


https://user-images.githubusercontent.com/32492649/144702280-89a3e645-1884-41cf-8e8b-e2212d3cefb6.mp4

[Video URL](https://youtu.be/j68kPhCX8aM)

Steps involved in the implementation.

1. Driving around (with the help of the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package) through the environment, and scanning it using laser and RGBD camera. 
2. The robot is able to localize itself, and the [amcl package](http://wiki.ros.org/amcl) helps it in that (after having carefully tuned the AMCL parameters). AMCL, or Adaptive Monte Carlo Localization is a probabilistic localization system for a robot moving in 2D environment, and uses a particle filter approach to keep track of the robot's pose w.r.t. the environment.
3. It generates a static map of the environment, using RTAB-Map, i.e. the Real-Time Appearance-Based Mapping, which is an RGB-D SLAM approach, based on global loop closure detector, with real-time constraints. The package is: [rtabmap_ros](http://wiki.ros.org/rtabmap_ros).
4. Next comes the navigation part. Here, the popular ROS package, [move_base](http://wiki.ros.org/move_base) has been used, and here again, we have to carefully tune the move_base parameters so that the robot is able to move around in the environment properly. This node links together a global and local planner to navigate through and reach a goal in the environment.
5. Lastly, a node has been created in this project, which gives 2 locations to the robot to navigate to, step, by step demonstrating the 'service' feature of the robot. The robot travels to a location where a 'marker' is set (akin to an object to be picked up), and 'dropped' to the appropriate location by the robot. 


#### Note

Before running the shell scripts, do:

```bash
catkin_make
source devel/setup.bash
```
from the `catkin_ws` directory.

### For running the project

- Clone the repo.
```bash
git clone git@github.com:amanarora9848/Building-Service-Bot.git
```

- Navigate to the `catkin_ws` directory and do a `catkin_make`
```bash
cd catkin_ws
catkin_make
```

- Source setup.bash
```bash
source devel/setup.bash
```

- To run the SLAM script, enter:
```bash
./src/scripts/test_slam.sh
```
https://user-images.githubusercontent.com/32492649/148048465-87f2a192-5f50-4547-bcda-45f9a58fc4ab.mp4

- To run the Navigation script, enter:
```bash
./src/scripts/test_navigation.sh
```

- To run the script to perform object delivery service, enter:
```bash
./src/scripts/home_service.sh
```

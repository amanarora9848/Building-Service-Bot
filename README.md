# Building-Service-Bot


##### This is a mini-project created to demonstrate a mobile service robot, capable of mapping its environment and autonomously navigating through it. It is capable of moving towards a certain goal, pick up an object, and drop it off at a certain other location.

Steps involved in the implementation.

1. Driving around (with the help of the [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard) package) through the environment, and scanning it using laser and RGBD camera. 
2. The robot is able to localize itself, and the [amcl package](http://wiki.ros.org/amcl) helps it in that. AMCL, or Adaptive Monte Carlo Localization is a probabilistic localization system for a robot moving in 2D environment, and uses a particle filter approach to keep track of the robot's pose w.r.t. the environment.
3. It generates a static map of the environment, using RTAB-Map, i.e. the Real_Time Appearance-Based Mapping, which is an RGBD SLAM approach, based on global loop closure detector, with real-time constraints. The package is: [rtabmap_ros](http://wiki.ros.org/rtabmap_ros).
4. Next comes the navigation part. Here, the popular ROS package, [move_base](http://wiki.ros.org/move_base) has been used. This node links together a global and local planner to navigate through and reach a goal in the environment.
5. Lastly, a node has been created in this project, which gives 2 locations to the robot to navigate to, step, by step demonstrating the 'service' feature of the robot. The robot travels to a location where a 'marker' is set (akin to an object to be picked up), and 'dropped' to the appropriate location by the robot. 


#### Note

Before running the shell scripts, do a `source devel/setup.bash` in the `catkin_ws` directory.
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initializing our pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map"; // Since the fixed frame is the map
  goal.target_pose.header.stamp = ros::Time::now();

  // This is the pickup position of the robot. Defining its position and orientation...
  goal.target_pose.pose.position.x = 4.17677974701;
  goal.target_pose.pose.position.y = -1.02883398;
  goal.target_pose.pose.position.z = 0.15230178833;

  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its pickup goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("##### ARRIVED AT PICKUP POINT #####");
  else
    ROS_INFO("The base failed to reach pickup point for some reason");

  // The Service Ferrari will pause for 5 seconds now.
  ros::Duration(5).sleep();

  // Now this is the dropoff position of the robot. Defining its position and orientation...
  goal.target_pose.pose.position.x = 4.00920200348;
  goal.target_pose.pose.position.y = -5.74286270142;
  goal.target_pose.pose.position.z = 0.15299790581;

  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its pickup goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("##### ARRIVED AT DROPOFF POINT #####");
  else
    ROS_INFO("The base failed to reach dropoff point for some reason");

  return 0;
}

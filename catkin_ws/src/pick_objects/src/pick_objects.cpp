#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// class Goal

int main(int argc, char** argv){
  
  // Initializing our pick_objects node
  ros::init(argc, argv, "pick_objects");
  
  // Creating the ROS NodeHandle object
  ros::NodeHandle n;
  
  // Creating ROS publisher
  ros::Publisher goal_pub = n.advertise<std_msgs::UInt8>("target", 10);
  
  // Creating a bool variable to keep track of the Service Ferrari.
  std_msgs::UInt8 track;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "odom"; // Since the fixed frame is the odom
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
  {
    ROS_INFO("##### ARRIVED AT PICKUP POINT #####");
    track.data = 1;
    goal_pub.publish(track);
  } 
  else
    ROS_INFO("The base failed to reach pickup point for some reason");

  // The Service Ferrari will pause for 5 seconds now.
  ros::Duration(5).sleep();

  // Now this is the dropoff position of the Ferrari. Defining its position and orientation...
  goal.target_pose.pose.position.x = 4.00920200348;
  goal.target_pose.pose.position.y = -5.74286270142;
  goal.target_pose.pose.position.z = 0.15299790581;

  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its dropoff goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("##### ARRIVED AT DROPOFF POINT #####");
    track.data = 2;
    goal_pub.publish(track);
  }
  else
    ROS_INFO("The base failed to reach dropoff point for some reason");
  
  // Wait for 5 seconds
  ros::Duration(5).sleep();

  // Send final message!
  ROS_INFO("The service Ferrari delivered the object successfully.");
  
  return 0;
}

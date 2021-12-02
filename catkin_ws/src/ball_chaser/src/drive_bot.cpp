#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"


class DriveBot
{
    private:
    // ROS::Publisher motor commands;
    ros::Publisher motor_command_publisher;
    ros::ServiceServer service;
    
    public:
    DriveBot(ros::NodeHandle *n)
    {
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        motor_command_publisher = n->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
        service = n->advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);
    }
    
    bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
                              ball_chaser::DriveToTarget::Response& resp)
    {    
				// This function should publish the requested linear x and angular velocities to the robot wheel joints
        ROS_INFO("DriveToTarget Request received - linear:%1.2f, angular:%1.2f", (float)req.linear_x, (float)req.angular_z);
        
        // Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;

        // Set wheel velocities, publish the requested velocities instead of constant values
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;

        // Publish angles to drive the robot
        motor_command_publisher.publish(motor_command);

        // Return a response message
        resp.msg_feedback = "Velocities set to - linear: " + std::to_string(motor_command.linear.x) + " and angular: " + std::to_string(motor_command.angular.z);
        ROS_INFO_STREAM(resp.msg_feedback);

        return true;
    }
};

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Instantiate the DriveBot class and do the deed! CleanCodeRocks!
    DriveBot drive_it = DriveBot(&n);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

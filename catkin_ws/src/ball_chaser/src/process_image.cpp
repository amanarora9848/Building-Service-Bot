#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


class ProcessImage
{
    private:
    // Define a client that can request services
    ros::ServiceClient client;
    ros::Subscriber sub1;

    public:
    ProcessImage(ros::NodeHandle *n)
    {
        // Define a client service capable of requesting services from command_robot
        client = n->serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
        sub1 = n->subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);
    }

    // This function calls the command_robot service to drive the robot in the specified direction
    void drive_robot(float lin_x, float ang_z)
    {
        // Request a service and pass the velocities to it to drive the robot
        // ROS_INFO_STREAM("Moving Ferrari so that it keeps chasing the white ball trophey...");

        // Pass the linear and angular velocities from our service after placing request, to drive the robot.
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        if(!client.call(srv))
        {
            ROS_ERROR("Failed to call the service drive_bot... Aborting in 3...2...1...");
        }
    }

    // This callback function continuously executes and reads the image data
    void process_image_callback(const sensor_msgs::Image img)
    {
        int white_pixel = 255;
        bool is_object_in_view;
        int found;
				
		// Traverse through all pixels and find if there's a white object.
        for (int i = 0; i < img.height * img.step; i++)
        {
            if (img.data[i] - white_pixel == 0 && 
                img.data[i+1] - white_pixel == 0 && 
                img.data[i+2] - white_pixel == 0)  
            {
                is_object_in_view = true;
                found = i;
                break;
            }
            else
            {
                is_object_in_view = false;
            }
        }
        if(is_object_in_view)
        {
            int left = fabs(img.width / 3);
            int right = fabs((2 * img.width) / 3);
            if (found % img.width < left) // If white pixel falls in left section of image
            {
                ROS_INFO_STREAM("Moving left");
                drive_robot(0.0, 0.5);
            }
            if (found % img.width >= right) // If white pixel falls in right section of image
            {
                ROS_INFO_STREAM("Moving right");
                drive_robot(0.0, -0.5);
            }
            if (found % img.width >= left && found % img.width < right) // If white pixel falls in middle section of image
            {
                ROS_INFO_STREAM("Moving straight ahead");
                drive_robot(0.5, 0.0);
            }
        }
        else if(!is_object_in_view) // If white pixel isn't there in the field of view.
        {
            ROS_INFO("Stopped. Can't see it. Sorry.");
            drive_robot(0.0, 0.0);
        }
    }
};


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ProcessImage chase_ball = ProcessImage(&n);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

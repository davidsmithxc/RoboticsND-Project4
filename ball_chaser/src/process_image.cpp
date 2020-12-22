#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

/*
class ProcessImage
{
public:
    // ctor
    ProcessImage()
    {
        // set up client for driving robot
        this->client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        // subscribe to camera
        this->sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &ProcessImage::process_image_callback, this);

        // spin
        // ProcessImage::drive_robot(0.0, 0.5);
    }

    void process_image_callback(const sensor_msgs::Image img)
    {
        int white_pixel = 255;
        float linear_x = 0.0;
        float angular_z = 0.5;

        // TODO: Loop through each pixel in the image and check if there's a bright white one
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        // Depending on the white ball position, call the drive_bot function and pass velocities to it
        // Request a stop when there's no white ball seen by the camera
        ROS_INFO_STREAM("Got img callback");
        for (int i = 0; i < (img.height * img.step); i++)
        {
            if (img.data[i] == white_pixel)
                ROS_INFO_STREAM("Ball seen...");
                angular_z = 0.5;
                break;
        }

        ProcessImage::drive_robot(linear_x, angular_z);

    }

    void drive_robot(float lin_x, float ang_z)
    {
        // Make a service call with the defined parameters
        srvDriveToTarget_.request.linear_x = lin_x;
        srvDriveToTarget_.request.angular_z = ang_z;

        // Call the safe_move service and pass the requested joint angles
        if (!client_.call(srvDriveToTarget_))
            ROS_ERROR("Failed to call service DriveToTarget");
    }


private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::ServiceClient client_;
    ball_chaser::DriveToTarget srvDriveToTarget_;
}; // end ProcessImage class
*/

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
        // Make a service call with the defined parameters
        ball_chaser::DriveToTarget srv;
        srv.request.linear_x = lin_x;
        srv.request.angular_z = ang_z;

        // Call the safe_move service and pass the requested joint angles
        if (!client.call(srv))
            ROS_ERROR("Failed to call service DriveToTarget");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
        static const float BYTES_PER_PIXEL = 3.0;
        int white_pixel = 255;
        float linear_x = 0.0;
        float angular_z = 0.25;

        // TODO: Loop through each pixel in the image and check if there's a bright white one
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        // Depending on the white ball position, call the drive_bot function and pass velocities to it
        // Request a stop when there's no white ball seen by the camera
        for (int i = 0; i < (img.height * img.step); i += 3)
        {
            if (img.data[i] == white_pixel &&
                img.data[i+1] == white_pixel &&
                img.data[i+2] == white_pixel)
                {
                    float position = 2.0 * ((float)(i % img.step) / BYTES_PER_PIXEL / (float)img.width - 0.5); // center around zero
                    ROS_INFO_STREAM("Ball seen: " + std::to_string(position*100));
                    linear_x = 0.5;
                    angular_z = position * -0.5; // flip sign to get correct rotational direction
                    break;
                }
        }

        drive_robot(linear_x, angular_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    /* use with class        
    ROS_INFO_STREAM("Starting process_image class node...");
    ProcessImage ProcessImageSAP;
    */

    ROS_INFO_STREAM("Starting process_image function node...");
    
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // spin!
    // drive_robot(0.0, 0.0);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
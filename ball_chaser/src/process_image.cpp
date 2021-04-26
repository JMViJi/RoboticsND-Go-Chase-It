#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // DONDE: Request a service and pass the velocities to it to drive the robot

    ROS_INFO_STREAM("Moving robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    int white_flag = 0;
    int part=0;
    float x=0.0;
    float y=0.0;
    float division=0.0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // Searching on the lines
    for (int i = 0; i < img.height; i++) {

        // Let see if the ball is on camera
        // Searching on the rows
        for (int j = 0; j < img.step; j++) {

            if (img.data[i * img.step + j] == white_pixel) {
                white_flag=1;
                division=j/3;
                break;
            }
        
        }
            
        if(white_flag==1) break;
    }

    part=0;
    if (division<img.width/3)        part=1;
    if (division>img.width*2/3)     part=2;
    if (division<=img.width*2/3 && division>=img.width/3)     part=3;




    if (white_flag==1){

        //If so identify the are in wich the ball falls
        switch(part){

            //Falls on the left
            case 1:
            x=0.1;
            y=0.1;
            break;

            //Falls on the right
            case 2:
            x=0.1;
            y=-0.1;
            break;

            //Falls on the center
            case 3:
            x=0.1;
            y=0.0;
            break;

            //Try to find the ball turning
            default:
            break;
        }
    }else{

            x=0.0;
            y=0.0;

    }

       
    drive_robot(x,y);


    // Ups! The ball is not on camera - lets make robot turn a bit to take a look --- IN PROCESS
    // Now we are going to move the ball on gazebo, but we can try to make the roobot try to find it.

    // Ups! The ball is not on camera - lets make robot turn a bit to take a look --- IN PROCESS
    // Now we are going to move the ball on gazebo, but we can try to make the roobot try to find it.

    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

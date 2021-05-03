#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // DONE: Request a service and pass the velocities to it to drive the robot

    ROS_INFO_STREAM("Moving robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service");}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int RGB_R = 255,RGB_G = 255,RGB_B = 255;
    int white_flag = 0;
    int part=0;
    float x=0.0;
    float z=0.0;
    float prefixed_vel_x = 0.1, prefixed_ang_z = 0.3;
    float division=0.0;

    // DONE: Loop through each pixel in the image and check if there's a bright white one    

    for (int i = 0; i < img.height; i++) {

        for (int j = 0; j < img.step; j++) {

            if (img.data[i * img.step + j] == RGB_R && img.data[i * img.step + j+1] == RGB_G && img.data[i * img.step + j+2] == RGB_B) {

                white_flag=1;
                division=j/3;

                // Then, identify if this pixel falls in the left, mid, or right side of the image
                if (division<img.width/3)                                   part=1;
                if (division>img.width*2/3)                                 part=2;
                if (division<=img.width*2/3 && division>=img.width/3)       part=3;

                break;
            }
        
        }
            
        if(white_flag==1) break;
    }


    if (white_flag==1){

        //If so identify the area in wich the ball falls
        switch(part){

            //Falls on the left
            case 1:
            x=prefixed_vel_x;
            z=prefixed_ang_z;
            break;

            //Falls on the right
            case 2:
            x=prefixed_vel_x;
            z=-prefixed_ang_z;
            break;

            //Falls on the center
            case 3:
            x=prefixed_vel_x;
            z=0.0;
            break;

            default:
            break;
        }
    }else{

            // Request a stop when there's no white ball seen by the camera            
            x=0.0;
            z=0.0;

    }

    // Depending on the white ball position, call the drive_bot function and pass velocities to it       
    drive_robot(x,z);
   
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
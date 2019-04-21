#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <iostream>

using namespace std;
// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Requesting velocities");

    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    //bool white_ball_found=false;
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
     //bool ball_detected = false;

    // Loop through each pixel in the image and check if its equal to the first one

// Image size is 800 (height) by 800 (width). The step size is 2400 -> each row contains rgb value per pixel
// left boundary=266 (800/3); right boundary=533 etc.
int count=0; //count of white pixels
float avg_index=0.0;
int x_index;
int position=0; //-1:left, 0:center, 1:right
for(int j=0; j<img.height*img.width; j++)
{
    int k=3*j; //there are rgb (3) values per "j"
    if(img.data[k]==white_pixel && img.data[k+1]==white_pixel && img.data[k+2]==white_pixel)
    {
        x_index=j%img.width;
        // calculate running average of all indices to follow the white ball
        avg_index=(x_index+avg_index*count)/(count+1);
        count++;
    }
}
if(count>0)
    {white_ball_found=true;
    cout<<"whiteball found on:"<<endl;
    if(avg_index<=266)
    {position=-1;
    cout<<"LEFT side"<<endl;
    drive_robot(0.2,-0.05);
    }
    else if(avg_index>=533)
    {position=1;
    cout<<"RIGHT side"<<endl;
    drive_robot(0.2,0.05);
    }
    else
    {cout<<"CENTER"<<endl;
    drive_robot(0.2,0);}
    }
else
    drive_robot(0.0,0.0);

    //cout<<"image_height:"<<img.height<<", img.width:"<<img.width<<", img.step:"<<img.step<<endl;
    //cout<<"image data [10]:"<<img.data[10]<<endl;

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

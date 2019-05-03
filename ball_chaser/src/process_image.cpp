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
    
// Image size is 800 (height) by 800 (width). The step size is 2400 -> each row contains rgb value per pixel
// left boundary=266 (800/3); right boundary=533 etc.
int count=0; //count of white pixels
float avg_index=0.0;
int x_index {};
int position=0; //-1:left, 0:center, 1:right

// Following for loop calculates a center of geometry of white pixel area
// First,white pixel is identified, and it's x axis position/index is captured.
// Then, a running average of all indices is maintained to calculate new avg_index everytime
// ,i.e. avg_index=(x_index+avg_index*count)/(count+1);
// The avg_index indicates x index of center of geometry, for instance- center of a ball! 
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

// As described earlier, the count indicates if at least one single white pixel was identified
// The average index indicated x index/position of center of geometry of white pixel area.
// Since the pixel width is 800, the left boundary spans 0 to 266 and right boundary from 533 to 800.
if(count>0)
    {//white_ball_found=true;
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

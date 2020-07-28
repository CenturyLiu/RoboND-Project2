#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// Define different modes for the robot
enum MODE {stop, straight, turn};

MODE mode = stop; // robot is initially at rest


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // call the command_robot service
    if(!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera


    // get the left/right boundary
    unsigned int left_boundary = img.width / 2 - 25 ;
    unsigned int right_boundary = img.width / 2 + 25;

    // get a wider boundary
    unsigned int left_boundary_wider = img.width / 4;
    unsigned int right_boundary_wider = img.width * 3 / 4;

    // loop through the image raw data,get the center of the white ball
    // inside the image
    int count = 0; // number of white pixels
    double col_sum = 0; // the column axis of the center
    double row_sum = 0; // the row axis of the center

    // loop
    for(int ii = 0;ii < int(img.height * img.step / 3);++ii){
        unsigned int r = img.data[3 * ii];
        unsigned int g = img.data[3 * ii + 1];
        unsigned int b = img.data[3 * ii + 2];

        // check whether the given pixel is white
        if( r == white_pixel && g == white_pixel && b == white_pixel){
            count += 1;
            col_sum += ii % img.width;
            row_sum += ii / img.width;
        }
    }

    // give different velocity commands based on the the position of the white ball
    if(count <= 50){
        // no white pixel in the image, request a stop
        drive_robot(0.0,0.0);
        mode = stop;
        ROS_INFO("----\nToo few white pixels exist, stop robot");
    }
    else{
        double col_avg = col_sum / count;// get the average column coordinate
        ROS_INFO("----\ndetected column center: %f", col_avg);



        if(col_avg <= (double)left_boundary){
            // ball to the left
            // check the current mode of the vehicle, then decide the control
            if(col_avg <= (double)left_boundary_wider){
                // outside largest tolerable boundary, turn immediately
                // ball inside the left region, request turning left, i.e. positive value in the angular_z
                drive_robot(0.0,0.25);
                mode = turn; // change the vehicle mode
                ROS_INFO("turn left");
            }
            else if(col_avg > (double)left_boundary_wider && mode == turn){
                // turning from outside of the boundary back
                // keep turning until robot is close enough to the center
                drive_robot(0.0,0.25);
                mode = turn; // change the vehicle mode
                ROS_INFO("turn left");
            }
            else{
                // vehicle is currently going straight and inside a tolerable region
                // keep going straight
                drive_robot(1.5,0.0);
                mode = straight; // change the vehicle mode
                ROS_INFO("go straight forward");
            }

        }
        else if(col_avg >= (double)right_boundary){
            // ball to the right
            // check the current mode of the vehicle, then decide the control
            if(col_avg >= (double)right_boundary_wider){
                // outside largest tolerable boundary, turn immediately
                // ball inside the right region, request turning right, i.e. negative value in the angular_z
                drive_robot(0.0, -0.3);
                mode = turn; // change the mode of the vehicle
                ROS_INFO("turn right");
            }
            else if(col_avg < (double)right_boundary_wider && mode == turn){
                // turning from outside of the boundary back
                // keep turning until robot is close enough to the center
                drive_robot(0.0, -0.25);
                mode = turn; // change the mode of the vehicle
                ROS_INFO("turn right");
            }
            else{
                // vehicle is currently going straight and inside a tolerable region
                // keep going straight
                drive_robot(1.5,0.0);
                mode = straight; // change the vehicle mode
                ROS_INFO("go straight forward");
            }

        }
        else{
            // ball in center region, go straight forward
            drive_robot(1.5,0.0);
            mode = straight; // change the vehicle mode
            ROS_INFO("go straight forward");
        }
    }

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 0, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "ball_chaser/ArmToTarget.h"
#include "ball_chaser/LidToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// Define global clients for moving the shovel
ros::ServiceClient arm_client, lid_client;

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

// This function calls the move_arm service to move the shovel arm to specified location
void move_shovel_arm(float left_arm_angle, float right_arm_angle){
    ball_chaser::ArmToTarget srv;
    srv.request.left_arm = left_arm_angle;
    srv.request.right_arm = right_arm_angle;

    if(!arm_client.call(srv))
        ROS_ERROR("Failed to call service move_arm");
}

// This function calls the move_lid service to move the shovel lid to specified location
void move_shovel_lid(float left_lid_pos, float right_lid_pos){
    ball_chaser::LidToTarget srv;
    srv.request.left_lid = left_lid_pos;
    srv.request.right_lid = right_lid_pos;

    if(!lid_client.call(srv))
        ROS_ERROR("Failed to call service move_lid");
}

// This function reset the shovel
void reset_shovel(){
    // arm reset at 0 position
    move_shovel_arm(0.0,0.0);
    // lid open
    move_shovel_lid(-0.4,-0.4);
}


// This function executes a series of commands to catch the white ball and then release it
void catch_ball(){
    // open the lid
    move_shovel_lid(-0.4,-0.4);
    // drop the shovel arms
    move_shovel_arm(1.57,1.57);
    // close the lid
    move_shovel_lid(0.0,0.0);
    // raise the shovel arms to release the ball
    move_shovel_arm(-0.785,-0.785);
    // reset the shovel
    reset_shovel();
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
        double row_avg = row_sum / count; // get the average row coordinate
        ROS_INFO("----\ndetected column center: %f", col_avg);



        if(col_avg <= (double)left_boundary){
            // ball to the left
            // check the current mode of the vehicle, then decide the control
            if(col_avg <= (double)left_boundary_wider){
                // outside largest tolerable boundary, turn immediately
                // ball inside the left region, request turning left, i.e. positive value in the angular_z
                drive_robot(0.0,0.3);
                mode = turn; // change the vehicle mode
                ROS_INFO("turn left");
            }
            else if(col_avg > (double)left_boundary_wider && mode == turn){
                // turning from outside of the boundary back
                // keep turning until robot is close enough to the center
                drive_robot(0.0,0.3);
                mode = turn; // change the vehicle mode
                ROS_INFO("turn left");
            }
            else{
                // vehicle is currently going straight and inside a tolerable region
                // keep going straight
                drive_robot(0.4,0.0);
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
                drive_robot(0.0, -0.3);
                mode = turn; // change the mode of the vehicle
                ROS_INFO("turn right");
            }
            else{
                // vehicle is currently going straight and inside a tolerable region
                // keep going straight
                drive_robot(0.4,0.0);
                mode = straight; // change the vehicle mode
                ROS_INFO("go straight forward");
            }

        }
        else{

            // going straight with the ball in the center, check whether the ball is close enough
            if (count >= 11300 && count <= 18800){
                // close enough, stop robot, catch ball
                drive_robot(0.0,0.0);
                mode = straight; // change the vehicle mode
                ROS_INFO("Ball within ideal region, stop robot, catch ball");
                catch_ball();
            }
            else if(count > 18800 || row_avg > 580){
                // too close to the ball, slowly move backward
                drive_robot(-0.1,0.0);
                mode = straight; // change the vehicle mode
                ROS_INFO("Ball too close, slowly move back!");
            }
            else{
                // ball in center region, go straight forward
                drive_robot(0.4,0.0);
                mode = straight; // change the vehicle mode
                ROS_INFO("go straight forward");
            }

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

    // Define a client service capable of requesting services from move_arm
    arm_client = n.serviceClient<ball_chaser::ArmToTarget>("/ball_chaser/move_arm");

    // Define a client service capable of requesting services from move_arm
    lid_client = n.serviceClient<ball_chaser::LidToTarget>("/ball_chaser/move_lid");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 1, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

# RoboND-Project2
Implementation of Project 2 of [udacity robot software nanodegree](https://blog.udacity.com/2019/01/learn-robotics-engineering-program.html)

## Installation

Download the codes inside this repository, put the my_robot and ball_chaser package into the ROS workspace, then use catkin_make to build the project. For more 
details with respect to creating ROS workspace and installing ROS packages, follow instructions on [ROS tutorial](http://wiki.ros.org/ROS/Tutorials)

## Package structure
    .Project2                          # Go Chase It Project
        ├── my_robot                                 # my_robot package
        │   ├── config                               # config folder for configuration files
        |   |   ├── controllers.yaml                 # config file for gazebo controllers
        │   ├── launch                               # launch folder for launch files   
        │   │   ├── robot_control.launch             # launch file for robot control
        │   │   ├── robot_description.launch         # launch file for sending robot description 
        │   │   ├── world.launch                     # launch file for launching the world and spawn robot
        │   ├── meshes                               # meshes folder for sensors
        │   │   ├── hokuyo.dae
        │   ├── urdf                                 # urdf folder for xarco files
        │   │   ├── arm.xacro                        # the arm and shovel on the robot
        │   │   ├── ball_container.xacro             # the box on the robot
        │   │   ├── my_robot.xacro                   # the main file defining the robot
        │   │   ├── my_robot.gazebo                  # gazebo plugins for the robot, include gazebo_ros_control plugin, gazebo_diff_drive plugin and plugin for camera/lidar
        │   ├── world                                # world folder for world files
        │   │   ├── classroom_with_white_ball.world  # the world file for this project
        │   ├── CMakeLists.txt                       # compiler instructions
        │   ├── package.xml                          # package info
        ├── ball_chaser                              # ball_chaser package                   
        │   ├── launch                               # launch folder for launch files   
        │   │   ├── ball_chaser.launch               # the file for moving the robot and control the shovel
        │   │   ├── move_shovel.launch               # the file for moving the shovel, is included in the ball_chaser
        │   ├── src                                  # source folder for C++ scripts
        │   │   ├── drive_bot.cpp                    # service server for driving robot
        │   │   ├── move_arm.cpp                     # service server for moving the shovel arm
        │   │   ├── move_lid.cpp                     # service server for moving the shovel lid
        │   │   ├── process_images.cpp               # file for process image and call the other services
        │   ├── srv                                  # service folder for ROS services
        │   │   ├── ArmToTarget.srv
        │   │   ├── DriveToTarget.srv
        │   │   ├── LidToTarget.srv
        │   ├── CMakeLists.txt                       # compiler instructions
        │   ├── package.xml                          # package info                  
        └──         

## Function demo

The robot in this project is capable of 

 - chase the white ball
 - catch the white ball
    
![chase ball](https://github.com/CenturyLiu/RoboND-Project2/blob/master/chase_ball.gif)
> demo of robot chasing the white ball
    
    
![catch ball](https://github.com/CenturyLiu/RoboND-Project2/blob/master/catch_ball.gif)
> demo of robot catching the white ball

## Reference

The arm.xacro file inside my_robot/urdf is adapted from the file provided by the "ROS Deep Learning with TensorFlow 101" course from [Robot Ignite Academy](https://www.robotigniteacademy.com/) 

# YouRiBash Robot Tester

## Setup Instructions
Requires [ROS2](https://docs.ros.org/en/humble/Installation.html)

Open command line to the workspace directory

    cd ~/YouRiBash-Project

Download submodules

    git submodule update --init --remote

Install dependencies

    rosdep install --from-paths src -iy

Build the necessary packages

    colcon build --packages-up-to mtc_tutorial

Source the workspace

    source install/setup.bash

## Tester GUI
To use the GUI simply run **robot_tester_gui.py**. From here you can select which robots to test and which tasks to test them on. The GUI starts the MoveIt simulation through the *launch_moveit* function.

## MoveIt Integration
The *launch_moveit* function integrates with ROS2 to create a MoveIt simulation. It works by using a *LaunchService* to run launch files from various launch packages. The file **launch_moveit.py** can be run independently from the GUI for testing.

## MoveIt Task Constructor
Its possible to launch the MoveIt Task Constructor with the launch file pick_place_demo.launch.py located in the mtc_tutorial package (This has to be after swanning the robot because it takes the controller and the urdf of any nodes already working with Rviz)  

## TF Listener
Record the output data form the MoveIt task constructor node so it could be utilized to farther have the information about velocity or acceleration. Run it, after installing and sourcing the workspace from the launch file: mtc_demo.launch.py 

## Robot Configs
Each robot configuration package contains the necessary files and parameters to simulate and control a robot with MoveIt. They also contain launch files such as **demo.launch.py** which launches a simple RViz simulation containing the robot. These config packages were created using the MoveIt Setup Assistant.

How to run this package (so far):
    1. Install ROS2 and Gazebo Sim
    2. Change any reference to "sheneman" to your username (going to fix this later)
    3. cd to the top directory (i.e. /YouRiBash-Project/)
    4. Run "colcon build && source install/setup.bash"
    5. Run "python3 sim_control.py"

What this should do:
    Run an empty Gazebo sim called "test_world" and spawn a Franka Panda in the center. The script is set up such that you don't need to interact with ROS directly, so we can eventually move this functionality to the main script.

TODO:
    * Add grasping object, obstacles, target to the sim world
    * Let the user specify which robot to spawn
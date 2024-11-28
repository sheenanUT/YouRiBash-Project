import rclpy

from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

robots_directory = "/home/sheneman/YouRiBash-Project/robot-assets/urdfs/robots/"
robot_filenames = ["franka_panda/panda.urdf",
                   "kinova/kinova.urdf"]

# For loop to iterate through all robots files
for file in robot_filenames:
    # Use a launch file to start the gazebo simulation
    launch_service = LaunchService()
    launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': '/home/sheneman/YouRiBash-Project/test_world.sdf',
            }.items(),
        )
    )
    launch_service.include_launch_description(
        Node(
            # Spawn the robot
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-file", robots_directory + file,
                "-name", "robot",
            ]
        )
    )
    launch_service.run()

    # Shut everything down for the next test
    launch_service.shutdown()

    if file != robot_filenames[-1]:
        print("\nNext Robot? y/n")
        if input() != "y": break

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_filename = LaunchConfiguration("robot_filename")
    robot_filename_arg = DeclareLaunchArgument("robot_filename")

    return LaunchDescription([
        robot_filename_arg,
        IncludeLaunchDescription(
            # Start the gazebo simulation
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
        ),
        Node(
            # Spawn the robot
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-file", robot_filename,
                "-name", "robot",
            ]
        )
    ])
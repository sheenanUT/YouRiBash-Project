from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import shlex

def generate_launch_description():
    filename = "/home/sheneman/YouRiBash-Project/robot-assets/urdfs/robots/franka_panda/panda.urdf"
    init_pose_args = shlex.split("-x 0.0 -y 0.0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0")

    return LaunchDescription([
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
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            output="both",
            arguments=[
                "-file",
                filename,
                "-name",
                "robot",
                "-allow_renaming",
                "true",
            ]
            + init_pose_args
        )
    ])
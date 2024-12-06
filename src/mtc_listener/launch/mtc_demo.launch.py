from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='mtc_listener',
            executable='mtc_listener',
            name='listener',

        ),
    ])

from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder
import asyncio
import rclpy

robot_names = ["franka_panda",
               "kinova"]

def launch_moveit(robot_name, sim_type):
    package_name = robot_name + "_config"
    moveit_config = MoveItConfigsBuilder(robot_name, package_name=package_name).to_moveit_configs()

    # sim_type should be one of: basic, obs, elev, obs_elev
    mtc_file = "mtc_" + sim_type + "+.launch.py"

    # Use launch files to start the MoveIt suite
    sim_launch_service = LaunchService()

    # Move group node
    sim_launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(package_name),
                    'launch',
                    'move_group.launch.py'
                ])
            ])
        )
    )

    # Robot state publisher node
    sim_launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(package_name),
                    'launch',
                    'rsp.launch.py'
                ])
            ])
        )
    )

    # Spawn controllers
    sim_launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(package_name),
                    'launch',
                    'spawn_controllers.launch.py'
                ])
            ])
        )
    )
    
    # ROS2 control node
    sim_launch_service.include_launch_description(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                PathJoinSubstitution([
                    FindPackageShare(package_name),
                    'config',
                    'ros2_controllers.yaml'
                ])
            ],
        )
    )

    # MoveIt Task Constructor execution
    sim_launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mtc_tutorial'),
                    'launch',
                    mtc_file
                ])
            ])
        )
    )

    async def sim_task():
        task = asyncio.create_task(sim_launch_service.run_async())
        await asyncio.wait_for(task, 5)

    try:
        asyncio.run(sim_task())
    except asyncio.CancelledError:
        print("Cancelled task")

    # Shut everything down for the next test
    sim_launch_service.shutdown()
    print("Shut down service")

if __name__ == "__main__":
    launch_moveit("franka_panda", "basic")

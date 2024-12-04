from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

robot_names = ["franka_panda",
               "kinova"]

# For loop to iterate through all robots files
for name in robot_names:
    urdf_path = f"/home/sheneman/YouRiBash-Project/src/robot_configs/{name}_config/config/{name}.urdf"
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
                "-file", urdf_path,
                "-name", "robot",
            ]
        )
    )
    launch_service.run()

    # Shut everything down for the next test
    launch_service.shutdown()

    if name != robot_names[-1]:
        print("\nNext Robot? y/n")
        if input() != "y": break

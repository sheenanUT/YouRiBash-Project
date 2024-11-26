from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

robots_directory = "/home/sheneman/YouRiBash-Project/robot-assets/urdfs/robots/"
robot_filenames = ["franka_panda/panda.urdf",
                   "kinova/kinova.urdf"]
# For loop to iterate through all robots files
for file in robot_filenames:
    launch_service = LaunchService()
    launch_service.include_launch_description(
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('youribash_gz'),
                        'launch',
                        'sim.launch.py'
                    ])
                ]),
                launch_arguments={
                    'robot_filename': robots_directory + file,
                }.items(),
            )
    )
    launch_service.run()
    launch_service.shutdown
    print("\nNext Robot? y/n")
    if input() != "y": break

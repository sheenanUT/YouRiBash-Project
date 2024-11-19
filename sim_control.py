from launch import LaunchService
import sys

# This creates a direct path to the launch file
# Easier than trying to change the name of the launch folder or something
sys.path.insert(1, "/home/sheneman/YouRiBash-Project/src/youribash_gz/launch")
from simlaunch import generate_launch_description

launch_service = LaunchService()
launch_service.include_launch_description(generate_launch_description())
launch_service.run()
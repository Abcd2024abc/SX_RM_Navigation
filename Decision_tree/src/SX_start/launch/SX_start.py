import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory("SX_start"), "launch")

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "nav_start.py")
        ),
    )

    ld = LaunchDescription()
    ld.add_action(nav_launch)

    return ld
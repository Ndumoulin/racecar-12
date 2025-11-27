#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('racecar_behaviors')

    blob_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'blob_detection.launch.py')
        )
    )

    behaviors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'behaviors.launch.py')
        )
    )

    return LaunchDescription([
        blob_detection_launch,
        behaviors_launch
    ])

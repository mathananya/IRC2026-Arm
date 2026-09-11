"""
bringup.launch.py

Single entry point: camera + filters -> static TF (camera mount) -> laserscan.

The static transform is REQUIRED for pointcloud_to_laserscan's target_frame
("base_link") to resolve. Replace the xyz/rpy args below with your actual
camera mount offset once you've measured it on the physical rover. Until then
this is a placeholder so the pipeline runs end-to-end (e.g. against a bag or
in simulation) without TF errors.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_perception')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'realsense_filtered.launch.py')
        )
    )

    # PLACEHOLDER mount transform: base_link -> camera_link
    # args: x y z yaw pitch roll parent_frame child_frame  (meters / radians)
    # TODO: replace with measured offset from your rover CAD / tape measure.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '0.2', '0', '0.3',    # x y z: 20cm forward, 30cm up from base_link -- PLACEHOLDER
            '0', '0', '0',        # yaw pitch roll -- PLACEHOLDER (camera facing forward, level)
            'base_link', 'camera_link'
        ],
    )

    p2l_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'pointcloud_to_laserscan.launch.py')
        )
    )

    return LaunchDescription([
        camera_launch,
        static_tf,
        p2l_launch,
    ])
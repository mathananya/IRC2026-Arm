"""
pointcloud_to_laserscan.launch.py

Converts the filtered RealSense point cloud into a 2D sensor_msgs/LaserScan
for consumption by Nav2's obstacle layer.

Assumes realsense_filtered.launch.py is already running and publishing on
/camera/camera/depth/color/points (topic name depends on camera_name/namespace
and whether align_depth is enabled -- verify with `ros2 topic list` once the
camera is up).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cloud_topic = LaunchConfiguration('cloud_topic')

    declare_cloud_topic = DeclareLaunchArgument(
        'cloud_topic',
        default_value='/camera/camera/depth/color/points',
        description='Input point cloud topic from the RealSense driver. '
                    'Verify actual topic name with `ros2 topic list` once camera is running.'
    )

    config_path = os.path.join(
        get_package_share_directory('rover_perception'),
        'config',
        'pointcloud_to_laserscan.yaml'
    )

    p2l_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', cloud_topic),
            ('scan', '/scan'),
        ],
        parameters=[config_path],
        output='screen',
    )

    return LaunchDescription([
        declare_cloud_topic,
        p2l_node,
    ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
 
 
def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
 
    declare_camera_name = DeclareLaunchArgument(
        'camera_name', default_value='camera',
        description='Name of the camera node/namespace'
    )
    declare_camera_namespace = DeclareLaunchArgument(
        'camera_namespace', default_value='camera',
        description='Namespace for the camera'
    )
 
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'), '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name,
            'camera_namespace': camera_namespace,
 
            # --- Stream configuration ---
            'depth_module.profile': '640x480x30',
            'enable_color': 'true',
            'enable_depth': 'true',
            'align_depth.enable': 'true',   # align depth to color if you'll fuse both later
 
            # --- Point cloud output ---
            'pointcloud.enable': 'true',
            'pointcloud.ordered_pc': 'false',
 
            # TUNABLE PARAMETERS - retune these once camera is attached
 
            # 1. Decimation: reduces resolution -> fewer, cleaner points.
            #    filter_magnitude range 2-8. Start at 2 for 640x480; raise if
            #    you need more compute headroom.
            'decimation_filter.enable': 'true',
            'decimation_filter.filter_magnitude': '2',
 
            # 2. Threshold: clip depth range to your relevant nav volume.
            #    Drop anything closer than min_distance (sensor noise floor)
            #    or farther than max_distance (irrelevant to local planning).
            'threshold_filter.enable': 'true',
            'threshold_filter.min_distance': '0.2',
            'threshold_filter.max_distance': '5.0',
 
            # 3. Spatial filter (edge-preserving smoothing).
            #    smooth_alpha: 0-1, higher = more smoothing.
            #    smooth_delta: edge threshold in depth units; higher = more
            #    aggressive smoothing across what would be an edge.
            'spatial_filter.enable': 'true',
            'spatial_filter.filter_magnitude': '2',
            'spatial_filter.filter_smooth_alpha': '0.5',
            'spatial_filter.filter_smooth_delta': '20.0',
            'spatial_filter.holes_fill': '0',
 
            # 4. Temporal filter. CAUTION on a moving rover: this averages
            #    over frames, which can smear/ghost obstacles at your ego
            #    velocity. Start conservative (short persistency) and only
            #    increase if depth is genuinely noisy while stationary.
            'temporal_filter.enable': 'true',
            'temporal_filter.filter_smooth_alpha': '0.4',
            'temporal_filter.filter_smooth_delta': '20.0',
            'temporal_filter.persistence_control': '3',
 
            # 5. Hole-filling: fills invalid pixels from neighbors.
            #    0=fill_from_left, 1=farest_from_around, 2=nearest_from_around
            'hole_filling_filter.enable': 'true',
            'hole_filling_filter.holes_fill': '1',
 
        }.items(),
    )
 
    return LaunchDescription([
        declare_camera_name,
        declare_camera_namespace,
        realsense_launch,
    ])
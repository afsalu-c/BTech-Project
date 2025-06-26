from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # Package directories
    pkg_fp_darc = get_package_share_directory('fp_darc')
    pkg_rplidar_ros = get_package_share_directory('rplidar_ros')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # File paths
    urdf_path = os.path.join(pkg_fp_darc, 'urdf', 'fp_darc.urdf')
    slam_yaml = os.path.join(pkg_fp_darc, 'config', 'slam_params.yaml')
    rplidar_launch = os.path.join(pkg_rplidar_ros, 'launch', 'view_rplidar_c1_launch.py')
    slam_launch = os.path.join(pkg_slam_toolbox,'launch', 'online_async_launch.py')
    #rviz_config_path = os.path.join(pkg_fp_darc, 'config', 'view.rviz')

    # Read the URDF
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Odometry node
        Node(
            package='odometry',
            executable='ticks_to_odom',
            name='ticks_to_odom',
            output='screen'
        ),

        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     name='rplidar_c1',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB0',
        #         'frame_id': 'lidar_link'
        #     }],
        #     output='screen'
        # ),

        # SLAM Toolbox
        # Node(
        #     package='slam_toolbox',
        #     executable='sync_slam_toolbox_node',  # or 'online_async_node'
        #     name='slam_toolbox',
        #     parameters=[slam_yaml],
        #     output='screen'
        # ),

        # Include RPLIDAR launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            launch_arguments={
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'lidar_link'
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'params_file' : slam_yaml
            }.items()
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_path],
        #     output='screen'
        # ),
    ])

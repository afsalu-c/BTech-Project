from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('fp_darc')
    urdf_path = os.path.join(pkg_share, 'urdf', 'fp_darc.urdf')
    config_path = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{"robot_description": open(urdf_path).read()}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_joint_state_broadcaster',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager',
                '--param-file', os.path.join(config_path, 'joint_state_broadcaster.yaml'),
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawner_diff_cont',
            arguments=[
                'diff_cont',
                '--controller-manager', '/controller_manager',
                '--param-file', os.path.join(config_path, 'diff_drive_controller.yaml'),
            ],
            output='screen',
        ),
    ])

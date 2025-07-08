from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Path to the URDF file
    # urdf_file_name = 'fp_darc.urdf'  # <-- change this to your actual URDF file name
    # urdf_path = os.path.join(
    #     get_package_share_directory('fp_darc'),  # <-- change this to your package name
    #     'urdf',
    #     urdf_file_name
    # )
    xacro_file = os.path.join(
        get_package_share_directory('fp_darc'),
        'urdf',
        'sweebo.urdf.xacro'
        )
    
    sweebo_description = xacro.process_file(xacro_file).toxml()

    # with open(urdf_path, 'r') as infp:
    #     robot_description_content = infp.read()

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': sweebo_description}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

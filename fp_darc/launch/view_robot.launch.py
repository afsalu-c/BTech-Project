import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sweebo_description_dir = get_package_share_directory("fp_darc")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(sweebo_description_dir, "urdf", "sweebo.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )

    sweebo_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_sim:=False"
        ]),
        value_type=str
    )

    # ✅ Correct usage here
    rviz_config_path = os.path.join(sweebo_description_dir, "rviz", "view_urdf.rviz")

    return LaunchDescription([
        model_arg,
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
            output='screen',
            arguments=["-d", rviz_config_path],  # ✅ Use correct variable
        )
    ])

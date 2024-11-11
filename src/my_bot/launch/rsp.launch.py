import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the path to the XACRO file
    urdf_file_path = os.path.join(
        get_package_share_directory('my_bot'),
        'description',
        'robot.urdf.xacro'
    )

    # Process the XACRO file and wrap it in a ParameterValue
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

    # Launch description
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),
    ])

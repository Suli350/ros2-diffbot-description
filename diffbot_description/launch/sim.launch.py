"""Robot model + kinematic simulator + RViz. Drive it with teleop or circle_driver."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('diffbot_description')
    xacro_file = os.path.join(share, 'urdf', 'diffbot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    demo = LaunchConfiguration('demo')

    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='false',
                              description='Drive a figure eight automatically'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='diffbot_description', executable='diff_drive_sim', output='screen',
             parameters=[os.path.join(share, 'config', 'diffbot.yaml')]),
        Node(package='diffbot_description', executable='circle_driver',
             parameters=[{'figure_eight': True}], condition=IfCondition(demo)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'sim.rviz')]),
    ])

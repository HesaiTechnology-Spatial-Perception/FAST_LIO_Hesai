from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share = get_package_share_directory('fast_lio')
    rviz_cfg = LaunchConfiguration('rviz_cfg')
    rviz_use = LaunchConfiguration('rviz')
    rviz_node = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg], condition=IfCondition(rviz_use))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_cfg', default_value=os.path.join(share, 'rviz', 'fastlio.rviz')),
        DeclareLaunchArgument('config_file', default_value=os.path.join(share, 'config', 'jt64p.yaml')),
        Node(package='fast_lio', executable='fastlio_mapping', parameters=[LaunchConfiguration('config_file')], output='screen'),
        rviz_node,
    ])

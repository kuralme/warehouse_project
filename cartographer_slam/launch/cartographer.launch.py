import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    rviz_config_sim = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'mapping.rviz')
    rviz_config_real = os.path.join(get_package_share_directory('cartographer_slam'), 'rviz', 'mapping_real.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    configuration_basename_sim = 'cartographer_sim.lua'
    configuration_basename_real = 'cartographer_real.lua'

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    load_nodes_real = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_sim_time])),
        actions=[
            Node(
                package='cartographer_ros', 
                executable='cartographer_node', 
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-configuration_directory', cartographer_config_dir,
                        '-configuration_basename', configuration_basename_real]),

            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                output='screen',
                name='occupancy_grid_node',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_real]
            )
        ]
    )

    load_nodes_sim = GroupAction(
        condition=IfCondition(use_sim_time),
        actions=[
            Node(
                package='cartographer_ros', 
                executable='cartographer_node', 
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-configuration_directory', cartographer_config_dir,
                        '-configuration_basename', configuration_basename_sim]),

            Node(
                package='cartographer_ros',
                executable='cartographer_occupancy_grid_node',
                output='screen',
                name='occupancy_grid_node',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_sim]
            )
        ]
    )

    # Declare the launch options
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)

    ld.add_action(load_nodes_real)
    ld.add_action(load_nodes_sim)

    return ld
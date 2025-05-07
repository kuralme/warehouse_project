import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter


def generate_launch_description() -> LaunchDescription:
    
    nav2_yaml_sim = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')
    nav2_yaml_real = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_real.yaml')
    rviz_config_sim = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localizer_sim.rviz')
    rviz_config_real = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localizer_real.rviz')

    map_file = LaunchConfiguration('map_file')
    use_real_robot = PythonExpression([
        "True if 'real' in '", map_file, "' else False"
    ])

    declare_use_real_robot = DeclareLaunchArgument(
        'use_real_robot',
        default_value='False',
        description='Use real robot settings if True, simulation if False',
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Name of the map YAML file inside map_server/config'
    )
    
    load_nodes_sim = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_real_robot])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'yaml_filename': PathJoinSubstitution([
                        FindPackageShare('map_server'),'config', map_file])
                }]
            ),
            
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_yaml_sim],
                remappings=[('/cmd_vel', '/diffbot_base_controller/cmd_vel_unstamped')]
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': ['map_server', 'amcl']}]
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

    load_nodes_real = GroupAction(
        condition=IfCondition(use_real_robot),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'yaml_filename': PathJoinSubstitution([
                        FindPackageShare('map_server'),'config', map_file])
                }]
            ),
            
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_yaml_real],
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': True},
                            {'autostart': True},
                            {'node_names': ['map_server', 'amcl']}]
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

    # Declare the launch options
    ld = LaunchDescription()
    ld.add_action(declare_map_file)
    ld.add_action(declare_use_real_robot)

    ld.add_action(load_nodes_real)
    ld.add_action(load_nodes_sim)

    return ld
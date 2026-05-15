from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('elevation_traversability')
    params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg, 'rviz', 'offline_pcd.rviz'])

    args = [
        DeclareLaunchArgument('pcd_file'),
        DeclareLaunchArgument('output_pgm',              default_value=''),
        DeclareLaunchArgument('frame_id',                default_value='map'),
        DeclareLaunchArgument('map_resolution',          default_value='0.15'),
        DeclareLaunchArgument('padding',                 default_value='2.0'),
        DeclareLaunchArgument('min_z',                   default_value='-1000000.0'),
        DeclareLaunchArgument('max_z',                   default_value='1000000.0'),
        DeclareLaunchArgument('max_height_above_ground', default_value='1000000.0'),
        DeclareLaunchArgument('noise_removal_neighbors', default_value='0'),
        DeclareLaunchArgument('rviz',                    default_value='true'),
    ]

    overrides = {
        'pcd_file':                LaunchConfiguration('pcd_file'),
        'output_pgm':              LaunchConfiguration('output_pgm'),
        'frame_id':                LaunchConfiguration('frame_id'),
        'map_resolution':          LaunchConfiguration('map_resolution'),
        'padding':                 LaunchConfiguration('padding'),
        'min_z':                   LaunchConfiguration('min_z'),
        'max_z':                   LaunchConfiguration('max_z'),
        'max_height_above_ground': LaunchConfiguration('max_height_above_ground'),
        'noise_removal_neighbors': LaunchConfiguration('noise_removal_neighbors'),
    }

    node = Node(
        package='elevation_traversability',
        executable='pcd_traversability_offline_node',
        name='pcd_traversability_offline',
        output='screen',
        parameters=[params, overrides],
        emulate_tty=True,
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_offline',
        output='screen', arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [node, rviz])

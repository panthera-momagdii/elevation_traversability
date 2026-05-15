from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Names of the YAML-overridable launch args. Default '' = leave YAML alone.
YAML_OVERRIDE_KEYS = ('map_frame', 'robot_frame', 'raw_topic')


def _setup(context, *_, **__):
    pkg = FindPackageShare('elevation_traversability')
    params = PathJoinSubstitution([pkg, 'config', 'params.yaml'])
    rviz_cfg = PathJoinSubstitution([pkg, 'rviz', 'elevation_traversability.rviz'])

    overrides = {}
    for k in YAML_OVERRIDE_KEYS:
        v = LaunchConfiguration(k).perform(context)
        if v != '':
            overrides[k] = v

    parameters = [params, overrides] if overrides else [params]

    node = Node(
        package='elevation_traversability',
        executable='elevation_traversability_node',
        name='elevation_traversability',
        output='screen',
        parameters=parameters,
        emulate_tty=True,
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_elevation_trav',
        output='screen', arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [node, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz',        default_value='true'),
        DeclareLaunchArgument('map_frame',   default_value=''),
        DeclareLaunchArgument('robot_frame', default_value=''),
        DeclareLaunchArgument('raw_topic',   default_value=''),
        OpaqueFunction(function=_setup),
    ])

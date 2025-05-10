import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, Action
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource

launch_args = [
    DeclareLaunchArgument('tree_name', default_value='TestLeader1Sam', description='Name of the behavior tree to load'),
    DeclareLaunchArgument('ns', description='Namespace for the agent.'),
]

def generate_launch_description():

    config = os.path.join(get_package_share_directory('formation_controller'), 'config', 'overall_params.yaml')

    ns = LaunchConfiguration('ns')
    tree_name = LaunchConfiguration('tree_name')

    leader_bt = Node(
        name='leader_bt',
        executable='bt_planner',
        package='tuper_btcpp',
        output='screen',
        namespace=ns,
        parameters=[config, {
            'xml_directory': os.path.join(get_package_share_directory('tuper_btcpp'), 'behavior_trees'),
            'tree_name': tree_name,  # Active tree_name
            'loop_rate': 20.0,
        }]
    )

    # Relay nodes for Unity with SAM
    relay_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sam_thruster_relay'), 'launch', 'relay_nodes.launch.py')
        )
    )
    relay_nodes = GroupAction([PushRosNamespace(ns), relay_nodes])

    return LaunchDescription([
        *launch_args,
        leader_bt,
        relay_nodes,
    ])
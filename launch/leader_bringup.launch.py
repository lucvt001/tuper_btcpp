import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    config = os.path.join(get_package_share_directory('formation_controller'), 'config', 'overall_params.yaml')

    tree_name_arg = DeclareLaunchArgument(
        'tree_name',
        default_value='Test',
        description='Name of the behavior tree to load'
    )

    # Subscribe to gps and heading topic of supreme leader. Broadcast world (utm) -> map, no translation, only rotation
    # Publish origin gps to /NS/origin_gps so that each agent can calculate its current local position, FLU frame
    origin_pub = Node(
        name='origin_pub',
        executable='origin_pub',
        package='formation_controller', parameters=[config]
    )

    # Listen to gps and heading of this agent. Calculate the local position relative to origin_gps
    # And then broadcast the transform world (utm) -> agent
    # Only used for agents on the surface with access to gps (aka leaders)
    gps_heading_to_tf = Node(
        name='gps_heading_to_tf',
        executable='gps_heading_to_tf',
        package='formation_controller', parameters=[config],
    )

    leader_bt = Node(
        name='leader_bt',
        executable='bt_planner',
        package='tuper_btcpp',
        output='screen',
        parameters=[{
            'xml_directory': os.path.join(get_package_share_directory('tuper_btcpp'), 'behavior_trees'),
            'tree_name': LaunchConfiguration('tree_name'),  # Active tree_name
            'loop_rate': 20
        }]
    )
    leader_bt = TimerAction(period=7.0, actions=[leader_bt])

    # Relay nodes for Unity with SAM
    relay_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sam_thruster_relay'), 'launch', 'relay_nodes.launch.py')
        )
    )

    return LaunchDescription([
        tree_name_arg,
        origin_pub,
        gps_heading_to_tf,
        relay_nodes,
        leader_bt,
    ])

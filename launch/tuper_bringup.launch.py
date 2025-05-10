import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Contains use_sim_time param
    config = os.path.join(get_package_share_directory('formation_controller'), 'config', 'overall_params.yaml')

    # Subscribe to leader1 GPS and publish the first GPS received as the origin for all GPS computations
    origin_pub = Node(
        name='origin_pub',
        executable='origin_pub',
        package='arduagent',
        namespace='leader1',
        output='screen',
        parameters=[config, {
            'input_topic': 'core/gps',
            'output_topic': '/origin_gps'
        }]
    )

    # Leaders
    leader1_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tuper_btcpp'), 'launch', 'leader_bringup.launch.py')
        ), launch_arguments={
            'tree_name': 'TestLeader1Sam',
            'ns': 'leader1',
        }.items()
    )
    leader2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tuper_btcpp'), 'launch', 'leader_bringup.launch.py')
        ), launch_arguments={
            'tree_name': 'TestLeader2Sam',
            'ns': 'leader2',
        }.items()
    )

    # Temporary node
    map_gt = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "world", "map_gt"],
    )

    ping_synchronizer = Node(
        name='ping_synchronizer',
        package='tuper_sim_utils',
        executable='ping_synchronizer',
        output='screen',
        parameters=[config, {
            'leader1_acoustic_topic': '/leader1/acoustic/write',
            'leader2_acoustic_topic': '/leader2/acoustic/write',
        }]
    )

    leader1_string_relay = Node(
        name='leader1_string_relay',
        executable='transform',
        package='topic_tools',
        output='screen',
        arguments="/leader1/acoustic/write/string --field data /leader1/acoustic/write smarc_msgs/StringStamped smarc_msgs.msg.StringStamped(data=m) --import std_msgs smarc_msgs --wait-for-start".split()
    )

    return LaunchDescription([    
        origin_pub,
        leader1_string_relay,
        TimerAction(period=0.0, actions=[map_gt]),
        TimerAction(period=0.2, actions=[ping_synchronizer]),
        TimerAction(period=1.0, actions=[leader1_bringup]),
        TimerAction(period=1.5, actions=[leader2_bringup]),
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    foxglove = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    agent0_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('tuper_btcpp'), 'launch', 'leader_bringup.launch.py')
        ), launch_arguments={
            'tree_name': 'Test',
        }.items()
    )
    agent0_bringup = TimerAction(period=1.0, actions=[PushRosNamespace('agent0'), agent0_bringup])

    # agent1_bringup = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('formation_controller'), 'launch', 'agent_bringup.launch.py')
    #     ), launch_arguments={
    #         # 'namespace': 'agent1',
    #         'use_gps': 'True',
    #         'is_unity_sam': 'True',
    #     }.items()
    # )
    # agent1_bringup = GroupAction(actions=[PushRosNamespace('agent1'), agent1_bringup])

    record_bag = TimerAction(
        period=0.5,  # Delay in seconds
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-a'],  # Record all topics
            )
        ]
    )

    # Temporary node
    map_gt = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "-1.57", "0", "0", "world", "map_gt"],
    )

    return LaunchDescription([    
        foxglove,
        agent0_bringup,
        map_gt,
        # record_bag,
    ])

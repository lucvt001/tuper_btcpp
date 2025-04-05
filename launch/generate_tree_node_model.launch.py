import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tuper_btcpp',
            executable='generate_tree_node_model',
            name='generate_tree_node_model',
            output='screen',
            parameters=[{
                'tree_node_model_xmlfile_path': os.path.join(get_package_share_directory('tuper_btcpp'), 'behavior_trees', 'new_node.xml'),
            }]
        )
    ])

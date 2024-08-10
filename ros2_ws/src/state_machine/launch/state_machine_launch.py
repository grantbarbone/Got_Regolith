from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Battery Node
    Node(
            package='state_machine',
            executable='battery_subpub',
            name='battery_node',
            output='screen'
        ),

        # Graph Algorithm Node
    Node(
            package='state_machine',
            executable='graph_algorithm_subpub',
            name='graph_algorithm_node',
            output='screen'
        ),

        # Lidar Node
    Node(
            package='state_machine',
            executable='lidar_subpub',
            name='lidar_node',
            output='screen'
        ),

        # Main Node
    Node(
            package='state_machine',
            executable='main_subpub',
            name='main_node',
            output='screen'
        ),

        # Motor Controls Node
    Node(
            package='state_machine',
            executable='motor_controls_subpub',
            name='motor_controls_node',
            output='screen'
        ),

        # Radar Node
    Node(
            package='state_machine',
            executable='radar_subpub',
            name='radar_node',
            output='screen'
        ),

        # ZED Node
    Node(
            package='state_machine',
            executable='zed_subpub',
            name='zed_node',
            output='screen'
        )
])
    

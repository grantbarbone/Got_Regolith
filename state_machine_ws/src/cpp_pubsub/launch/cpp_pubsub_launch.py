import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),

        Node(
            package='cpp_pubsub',
            executable='graph_algorithm_pubsub',
            name='graph_algorithm_pubsub',
            output='screen',
            emulate_tty=True,
            # Pass environment variable for logging level
            env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{time}: [{name}] [{severity}] [{function}]: {message}',
                 'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED': '1',
                 'RCUTILS_LOGGING_BUFFERED_STREAM': '1'},
            # Arguments to control logging level based on passed launch argument
            remappings=[('graph_algorithm_pubsub', LaunchConfiguration('log_level'))],
        ),
        
        Node(
            package='cpp_pubsub',
            executable='sensor_state_machine',
            name='sensor_state_machine',
            output='screen',
            emulate_tty=True,
            # Pass environment variable for logging level
            env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{time}: [{name}] [{severity}] [{function}]: {message}',
                 'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED': '1',
                 'RCUTILS_LOGGING_BUFFERED_STREAM': '1'},
            # Arguments to control logging level based on passed launch argument
            remappings=[('sensor_state_machine', LaunchConfiguration('log_level'))],
        ),

        Node(
            package='cpp_pubsub',
            executable='lidar_pubsub',
            name='lidar_pubsub',
            output='screen',
            emulate_tty=True,
            # Pass environment variable for logging level
            env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{time}: [{name}] [{severity}] [{function}]: {message}',
                 'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED': '1',
                 'RCUTILS_LOGGING_BUFFERED_STREAM': '1'},
            # Arguments to control logging level based on passed launch argument
            remappings=[('lidar_pubsub', LaunchConfiguration('log_level'))],
        ),
    ])

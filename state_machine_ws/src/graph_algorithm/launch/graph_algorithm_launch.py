from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    urdf_file = LaunchConfiguration('urdf_file', default='~/state_machine_ws/src/graph_algorithm/urdf/rover.urdf')

    # Specify URDF file path
    urdf = get_package_share_directory('graph_algorithm') + '/urdf/rover.urdf'

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf,
            description='Absolute path to URDF file'
        ),

        # Launch nodes
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '~/state_machine_ws/src/graph_algorithm/rviz/my_robot_config.rviz'],
            output='screen'
        ),

        Node(
            package='graph_algorithm',
            executable='graph_algorithm_node',
            name='graph_algorithm_node',
            output='screen'
        )
    ])

# If running this file directly
if __name__ == '__main__':
    generate_launch_description()

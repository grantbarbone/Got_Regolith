import launch
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    base_dir = get_package_share_directory('python_applications')

    urdf_path = os.path.join(base_dir, 'radariq_sensor.urdf')
    rviz_config = os.path.join(base_dir, 'radariq_objects.rviz')

    sensor_model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[urdf_path]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    objects_node = Node(
        package='python_applications',
        executable='objects_publisher',
        name='objects_publisher',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'framerate': 10,  # Ensure this value is between 0 and 20
            'distancefilter_min': 0.0,
            'distancefilter_max': 100.0,
            'anglefilter_min': -3.14,
            'anglefilter_max': 3.14,
            'pointdensity': 10,
            'certainty': 50,
            'marker_topic': 'marker_topic',
            'object_data_topic': 'object_data_topic',
            'frame_id': 'base_link'
        }]
    )

    return launch.LaunchDescription([rviz_node, sensor_model, objects_node])
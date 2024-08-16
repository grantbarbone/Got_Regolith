from setuptools import setup
from glob import glob

package_name = 'python_applications'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),  # Updated for .launch.py
        ('share/' + package_name, glob('rviz/*.urdf')),
        ('share/' + package_name, glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'radariq'],
    zip_safe=True,
    maintainer='RadarIQ',
    maintainer_email='support@radariq.io',
    description='The RadarIQ ROS Package, supports the RIQ-M model',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_publisher = python_applications.pointcloud_publisher_node:main',
            'objects_publisher = python_applications.objects_publisher_node:main',
            'graph_algorithm_2d_subpub = python_applications.graph_algorithm:main',  # Existing entry point
            'radar_publisher = python_applications.radar:main'  # Reference your file here
        ],
    },
)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Buscamos la ruta automática a tu archivo YAML
    config_file = os.path.join(
        get_package_share_directory('person_tracker'),
        'config',
        'params.yaml'
    )

    object_processing_node = Node(
        package='person_tracker',
        executable='object_processing_main',
        name='object_processing_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
      object_processing_node,
  ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Buscamos la ruta automática a tu archivo YAML
    config_file = os.path.join(
        get_package_share_directory('person_tracker'),
        'config',
        'params.yaml'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Lanzar el nodo de marcadores para RViz si es true'
    )

    object_processing_node = Node(
        package='person_tracker',
        executable='object_processing_main',
        name='object_processing_node',
        parameters=[config_file, {'use_sim_time': True}],
        output='screen'
    )

    lidar_tracker_node = Node(
        package='person_tracker',
        executable='lidar_tracker_main',
        name='detections_tracking_node',
        parameters=[config_file, {'use_sim_time': True}],
        output='screen'
    )

    debug_node = Node(
        package='person_tracker',
        executable='tracker_debug_main',
        name='tracker_debug_node',
        parameters=[config_file, {'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('debug'))
    )

    return LaunchDescription([
      debug_arg,
      object_processing_node,
      lidar_tracker_node,
      debug_node
  ])

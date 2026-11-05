import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    model_arg = DeclareLaunchArgument(
        'model',
        default_value='yolov8m-pose.pt'
    )

    input_image_arg = DeclareLaunchArgument(
        'input_image_topic',
        default_value='/color/image'
    )

    tracker_arg = DeclareLaunchArgument(
        'tracker',
        default_value='src/perception-study/modified_trackers/mod_botsort.yaml'
    )

    with_reid_arg = DeclareLaunchArgument(
        'with_reid',
        default_value='False'
    )


    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolo_bringup'),
                'launch',
                'yolo.launch.py'
            ])
        ]),
        launch_arguments={
            'model': LaunchConfiguration('model'),
            'input_image_topic': LaunchConfiguration('input_image_topic'),
            'tracker': LaunchConfiguration('tracker'),
            'with_reid': LaunchConfiguration('with_reid'),
        }.items()
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

    convert_2d_3d = Node(
        package='person_tracker',
        executable='detection_2d_to_3d_depth_main',
        output='screen',
        parameters=[config_file, {'use_sim_time': True}],
        remappings=[
            ('input_depth', '/stereo/depth'),
            ('input_detection_2d', '/yolo/detections'),
            ('camera_info', '/color/camera_info'),
            ('output_detection_3d', 'detection_3d'),
        ]
    )


    return LaunchDescription([
      debug_arg,
      model_arg,
      input_image_arg,
      tracker_arg,
      with_reid_arg,
      yolo_launch,
      object_processing_node,
      lidar_tracker_node,
      debug_node,
      convert_2d_3d
  ])

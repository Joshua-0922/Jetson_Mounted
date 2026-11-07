"""Camera + YOLO detector.

    ros2 launch yolo_detector yolo.launch.py                 # camera + detector
    ros2 launch yolo_detector yolo.launch.py camera:=false   # detector only
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Ultralytics is installed in this venv (system site-packages on, so rclpy works).
YOLO_PYTHON = os.path.expanduser('~/yolo-venv/bin/python')


def generate_launch_description():
    args = {
        'model': ('', str), 'conf': ('0.25', float), 'iou': ('0.45', float),
        'imgsz': ('640', int), 'device': ('cuda:0', str), 'half': ('true', bool),
        'image_topic': ('/camera/image_raw', str), 'publish_annotated': ('true', bool),
        'best_effort': ('false', bool),
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in args.items()]
    decls.append(DeclareLaunchArgument('camera', default_value='true',
                                       description='also start imx219_camera'))
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=t)
              for k, (_, t) in args.items()}

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('imx219_camera'), 'launch', 'camera.launch.py')),
        launch_arguments={'best_effort': LaunchConfiguration('best_effort')}.items(),
        condition=IfCondition(LaunchConfiguration('camera')))

    detector = Node(package='yolo_detector', executable='detector_node',
                    name='yolo_detector', prefix=YOLO_PYTHON,
                    parameters=[params], output='screen')
    return LaunchDescription(decls + [camera, detector])

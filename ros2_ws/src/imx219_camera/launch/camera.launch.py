from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        'sensor_id': ('0', int), 'width': ('1280', int), 'height': ('720', int),
        'fps': ('30', int), 'flip_method': ('0', int),
        'best_effort': ('false', bool),
        # colour: Argus WB preset (3 = fluorescent) + software AWB
        'wbmode': ('3', int), 'saturation': ('1.0', float),
        'awb': ('grayworld', str), 'awb_alpha': ('0.1', float),
        'awb_max_gain': ('2.0', float),
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, (v, _) in args.items()]
    params = {k: ParameterValue(LaunchConfiguration(k), value_type=t)
              for k, (_, t) in args.items()}
    return LaunchDescription(decls + [
        Node(package='imx219_camera', executable='camera_node',
             namespace='camera', name='imx219_camera',
             parameters=[params], output='screen'),
    ])

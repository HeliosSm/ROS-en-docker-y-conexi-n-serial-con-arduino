from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def node(pkg, exe, reliability):
    return Node(
        package=pkg,
        executable=exe,
        name=exe,
        output='screen',
        parameters=[{'reliability': reliability}]
    )

def generate_launch_description():
    reliability_arg = DeclareLaunchArgument(
        'reliability',
        default_value='reliable',
        description='QoS reliability: reliable | best_effort'
    )
    rel = LaunchConfiguration('reliability')

    return LaunchDescription([
        reliability_arg,
        node('sensor_serial', 'sensor_node', rel),
        node('sensor_serial', 'processor_node', rel),
        node('sensor_serial', 'monitor_node', rel),
        node('sensor_serial', 'exporter_node', rel),
    ])

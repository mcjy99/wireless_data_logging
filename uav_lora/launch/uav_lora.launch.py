#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyTHS1',
        description='Serial port for LoRa communication'
    )
    
    spreading_factor_arg = DeclareLaunchArgument(
        'sf',
        default_value='7',
        description='LoRa spreading factor (7-12)'
    )
    
    tx_power_arg = DeclareLaunchArgument(
        'tp',
        default_value='14',
        description='LoRa transmission power in dBm'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the node'
    )

    # LoRa node
    lora_node = Node(
        package='uav_lora',  # Replace with your actual package name
        executable='uav_lora_node',   # Replace with your executable name
        name='uav_lora_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'port': LaunchConfiguration('port'),
            'sf': LaunchConfiguration('sf'),
            'tp': LaunchConfiguration('tp')
        }],
        output='screen',
        emulate_tty=True,
        remappings=[
            # Uncomment and modify if you want to remap topics
            # ('lora_info', 'uav/lora_info'),
        ]
    )

    return LaunchDescription([
        port_arg,
        spreading_factor_arg,
        tx_power_arg,
        namespace_arg,
        lora_node
    ])
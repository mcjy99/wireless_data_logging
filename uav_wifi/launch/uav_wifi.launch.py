from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    
    return LaunchDescription([       
        # WiFi info publisher
        Node(
            package='uav_wifi',
            executable='uav_wifi_node',
            namespace='uav_wifi',
            name='uav_wifi_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'interface': 'wlP1p1s0',
            }]
        ),
    ])
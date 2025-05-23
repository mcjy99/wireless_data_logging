import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
)
from launch.actions import ExecuteProcess
from launch_ros.descriptions import ComposableNode

zed_package = get_package_share_directory('zed_wrapper')
mavros_package = get_package_share_directory('mavros_bridge')
# robot_viewer_package = get_package_share_directory('uosm_robot_viewer')
# xacro_path = os.path.join(robot_viewer_package, 'urdf', 'dogpa.urdf.xacro')
xacro_path = os.path.join(
    zed_package,
    'urdf',
    'zed_descr.urdf.xacro'
)

zed_config_common = os.path.join(
    zed_package,
    'config',
    'common_stereo.yaml'
)
zed_config_camera = os.path.join(
    zed_package,
    'config',
    'zed2i.yaml'
)

px4_config_path = os.path.join(
    get_package_share_directory('mavros_bridge'),
    'config', 'px4_config.yaml'
)
px4_pluginlists_path = os.path.join(
    get_package_share_directory('mavros_bridge'),
    'config', 'px4_pluginlists.yaml'
)

def generate_launch_description():
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(xacro_path)
    xacro_command.append(' ')
    xacro_command.append('camera_name:=')
    xacro_command.append('zed2i')
    xacro_command.append(' ')
    xacro_command.append('camera_model:=')
    xacro_command.append('zed2i')

    # Create the composable node container
    container = ComposableNodeContainer(
        name='mavros_bridge_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ZED Component
            ComposableNode(
                package='zed_components',
                namespace='',
                plugin='stereolabs::ZedCamera',
                name='zed_node',
                parameters=[
                # YAML files
                  zed_config_common,  # Common parameters
                  zed_config_camera,  # Camera related parameters
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='mavros_bridge',
                plugin='uosm::mavros::MavrosBridgeComponent',
                name='mavros_bridge',
                parameters=[{
                  'publish_rate_hz' : 30.0,
                }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Robot State Publisher Component
            ComposableNode(
                package='robot_state_publisher',
                plugin='robot_state_publisher::RobotStatePublisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': Command(xacro_command),
                    'package_path': mavros_package
                }]
            ),
        ],
        output='screen',
    )

    # Mavros Node
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
        {    
            # 'fcu_url': '/dev/ttyTHS1:1000000',
            'fcu_url': '/dev/ttyACM0:2000000',
            #'gcs_url': 'udp://@192.168.230.101',
            'gcs_url': 'udp://@10.42.0.1',
            #'gcs_url': 'udp://@10.100.17.135',
            'tgt_system': 1,
            'tgt_component': 1,
            'fcu_protocol': "v2.0",
            'respawn_mavros': "false",
            'namespace': "mavros",
        }],
    )

    # Joint State Publisher
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )
    # Static Transform Publisher
    static_tf_bl_cam = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = ["0.08650", "0", "-0.0450", "0", "0.0872665", "0", "base_link", "zed2i_camera_link"],
        output="screen"
    )

    uav_wifi = Node(
            package='uav_wifi',
            executable='uav_wifi_node',
            namespace='uav_wifi',
            name='uav_wifi_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'interface': 'wlP1p1s0',
            }]
    )

    bag_record = ExecuteProcess(
    cmd=['ros2', 'bag', 'record', '--output', '/home/nvidia/uav_autonomy_ws/data/rssi-wifi',
            '/zed_node/left/image_rect_color',
            '/zed_node/left/camera_info',
            '/zed_node/right/image_rect_color',
            '/zed_node/right/camera_info',
            '/zed_node/path_odom',
            '/zed_node/odom',
            '/zed_node/point_cloud/cloud_registered',
            '/mavros/local_position/pose',
            '/uav_wifi/wifi_info'
            ],
    output='screen')

    return LaunchDescription([       
        # Nodes and containers
        container,
        mavros,
        jsp,
        uav_wifi,
        static_tf_bl_cam,
        bag_record
    ])

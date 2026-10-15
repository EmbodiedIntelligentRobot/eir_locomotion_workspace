"""Official-driver processing path; no device opens unless start_driver:=true."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('eir_loco_depth_camera')
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='false'),
        DeclareLaunchArgument('serial_number',
                              default_value=EnvironmentVariable('EIR_DEPTH_CAMERA_SERIAL', default_value=''),
                              description='Optional device serial; mutually exclusive with usb_port'),
        DeclareLaunchArgument('usb_port',
                              default_value=EnvironmentVariable('EIR_DEPTH_CAMERA_USB_PORT', default_value='4-3'),
                              description='Optional USB UID from orbbec_camera list_devices_node'),
        DeclareLaunchArgument('start_debug_viewers', default_value='false'),
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution(
            [share, 'config', 'pro_parkour_depth.yaml'])),
        # Scope the raw-driver config so it cannot replace the processor config.
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution(
                    [share, 'launch', 'gemini336_raw_test.launch.py'])),
                launch_arguments={
                    'serial_number': LaunchConfiguration('serial_number'),
                    'usb_port': LaunchConfiguration('usb_port'),
                    'config': PathJoinSubstitution([share, 'config', 'gemini336_raw_test.yaml']),
                }.items()),
        ], condition=IfCondition(LaunchConfiguration('start_driver'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [share, 'launch', 'depth_transport.launch.py'])),
            launch_arguments={
                'config': LaunchConfiguration('config'),
                'start_publisher': 'false', 'start_processor': 'true',
                'start_stack_publisher': 'true', 'start_viewer': 'false',
                'start_debug_viewers': LaunchConfiguration('start_debug_viewers'),
            }.items()),
    ])

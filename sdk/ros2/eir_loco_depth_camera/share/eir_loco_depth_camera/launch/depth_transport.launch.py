"""Launch the camera publisher, policy stack publisher and/or viewer.

Examples:
  ros2 launch eir_loco_depth_camera depth_transport.launch.py \
    config:=... start_viewer:=false
  ros2 launch eir_loco_depth_camera depth_transport.launch.py \
    start_publisher:=false start_stack_publisher:=true start_viewer:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    config = LaunchConfiguration("config")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("eir_loco_depth_camera"), "config", "aiq11_depth.yaml"]
                ),
                description="Optional ROS parameter YAML file",
            ),
            DeclareLaunchArgument("start_publisher", default_value="true"),
            DeclareLaunchArgument("start_stack_publisher", default_value="false"),
            DeclareLaunchArgument("start_viewer", default_value="false"),
            DeclareLaunchArgument(
                "start_debug_viewers",
                default_value="false",
                description="Open three diagnostic windows for raw, processed and full images",
            ),
            DeclareLaunchArgument(
                "start_processor",
                default_value="false",
                description="Subscribe to raw Image and publish processed 32FC1 Image",
            ),
            DeclareLaunchArgument(
                "backend",
                default_value="orbbec",
                description="Camera backend: orbbec, realsense, opencv or synthetic",
            ),
            DeclareLaunchArgument(
                "show_window",
                default_value="true",
                description="Show the OpenCV grayscale window in the viewer",
            ),
            DeclareLaunchArgument(
                "publish_normalized",
                default_value="false",
                description="Publish x86-normalized depth as 32FC1",
            ),
            Node(
                package="eir_loco_depth_camera",
                executable="depth_camera_publisher",
                name="depth_camera_publisher",
                output="screen",
                parameters=[config, {"backend": LaunchConfiguration("backend")}],
                condition=IfCondition(LaunchConfiguration("start_publisher")),
            ),
            Node(
                package="eir_loco_depth_camera",
                executable="depth_image_viewer",
                name="depth_image_viewer",
                output="screen",
                parameters=[
                    config,
                    {
                        "show_window": LaunchConfiguration("show_window"),
                        "publish_normalized": LaunchConfiguration("publish_normalized"),
                    },
                ],
                condition=IfCondition(LaunchConfiguration("start_viewer")),
            ),
            Node(
                package="eir_loco_depth_camera",
                executable="depth_image_viewer",
                name="depth_viewer_raw",
                output="screen",
                parameters=[
                    config,
                    {
                        "topic": "/gemini336_a/depth/image_raw",
                        "window_name": "Gemini336 raw 16UC1",
                        "input_is_normalized": False,
                        "crop_enabled": False,
                        "show_window": LaunchConfiguration("show_window"),
                    },
                ],
                condition=IfCondition(LaunchConfiguration("start_debug_viewers")),
            ),
            Node(
                package="eir_loco_depth_camera",
                executable="depth_image_viewer",
                name="depth_viewer_processed",
                output="screen",
                parameters=[
                    config,
                    {
                        "topic": "/gemini336_a/depth/image_processed",
                        "window_name": "Gemini336 processed 32x18",
                        "input_is_normalized": True,
                        "crop_enabled": False,
                        "show_window": LaunchConfiguration("show_window"),
                    },
                ],
                condition=IfCondition(LaunchConfiguration("start_debug_viewers")),
            ),
            Node(
                package="eir_loco_depth_camera",
                executable="depth_image_viewer",
                name="depth_viewer_processed_full",
                output="screen",
                parameters=[
                    config,
                    {
                        "topic": "/gemini336_a/depth/image_processed_full",
                        "window_name": "Gemini336 processed full 64x40",
                        "input_is_normalized": True,
                        "crop_enabled": False,
                        "show_window": LaunchConfiguration("show_window"),
                    },
                ],
                condition=IfCondition(LaunchConfiguration("start_debug_viewers")),
            ),
            Node(
                package="eir_loco_depth_camera",
                executable="depth_stack_publisher",
                name="depth_stack_publisher",
                output="screen",
                parameters=[config],
                condition=IfCondition(LaunchConfiguration("start_stack_publisher")),
            ),
            Node(
                package="eir_loco_depth_camera",
                executable="depth_image_processor",
                name="depth_image_processor",
                output="screen",
                parameters=[config],
                condition=IfCondition(LaunchConfiguration("start_processor")),
            ),
        ]
    )

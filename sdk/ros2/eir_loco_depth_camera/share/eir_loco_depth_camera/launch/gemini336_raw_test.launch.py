"""Start a host-connected Gemini 336; optionally select it by USB port or serial."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def validate_device_selection(context):
    serial = LaunchConfiguration("serial_number").perform(context).strip()
    port = LaunchConfiguration("usb_port").perform(context).strip()
    if serial and port:
        raise RuntimeError("Set only one of usb_port or serial_number; check EIR_DEPTH_CAMERA_* too")
    return []


def start_capture(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    # A detached launch must forward SIGINT even if its parent has a terminal.
    command = ["ros2", "launch", "--noninteractive", "orbbec_camera", "gemini_330_series.launch.py",
               "camera_name:=" + value("camera_name"), "config_file_path:=" + value("config")]
    for selector in ("serial_number", "usb_port"):
        selected = value(selector).strip()
        # Humble's ros2 CLI rejects empty launch arguments; use vendor defaults.
        if selected:
            command.append(selector + ":=" + selected)
    return [Node(package="eir_loco_depth_camera", executable="camera_supervisor",
                 name="camera_supervisor", output="screen", parameters=[{
                     "capture_command": command,
                     "raw_topic": "/" + value("camera_name") + "/depth/image_raw",
                     "frame_timeout_s": 5.0,
                     "stop_timeout_s": 5.0,
                     "restart_delay_s": 1.0,
                 }])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_number",
                default_value=EnvironmentVariable("EIR_DEPTH_CAMERA_SERIAL", default_value=""),
                description="Optional device serial; mutually exclusive with usb_port",
            ),
            DeclareLaunchArgument(
                "usb_port",
                default_value=EnvironmentVariable("EIR_DEPTH_CAMERA_USB_PORT", default_value="4-3"),
                description="Optional USB UID from orbbec_camera list_devices_node",
            ),
            OpaqueFunction(function=validate_device_selection),
            DeclareLaunchArgument("camera_name", default_value="gemini336_a"),
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("eir_loco_depth_camera"),
                        "config",
                        "gemini336_raw_test.yaml",
                    ]
                ),
            ),
            OpaqueFunction(function=start_capture),
        ]
    )

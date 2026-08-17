import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_foxglove(context):
    address = LaunchConfiguration("address").perform(context)
    port_text = LaunchConfiguration("port").perform(context)
    if not address:
        raise ValueError("address must not be empty")
    try:
        port = int(port_text)
    except ValueError as error:
        raise ValueError("port must be an integer in 1..65535") from error
    if port < 1 or port > 65535:
        raise ValueError("port must be an integer in 1..65535")
    bridge_launch = os.path.join(
        get_package_share_directory("foxglove_bridge"),
        "launch",
        "foxglove_bridge_launch.xml",
    )
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(bridge_launch),
            launch_arguments={"address": address, "port": str(port)}.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("address", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="8765"),
            OpaqueFunction(function=_launch_foxglove),
        ]
    )

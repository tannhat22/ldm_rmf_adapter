import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # launch_dir = os.path.join(get_package_share_directory('rosbridge_server'),'launch')

    return LaunchDescription(
        [
            # Run node
            # Rosbrige lan:
            Node(
                package="ldm_rmf_adapter",
                namespace="",
                executable="ldm_rmf_adapter",
                name="ldm_rmf_adapter",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "ldm_server_config": os.path.join(
                            get_package_share_directory("ldm_rmf_adapter"),
                            "config.yaml",
                        ),
                        "timeout": 10.0,
                    }
                ],
            ),
        ]
    )

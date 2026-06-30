"""Launch the generic object rendering pipeline node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the object rendering launch description."""
    return LaunchDescription([
        Node(
            package="object_rendering",
            executable="scene_renderer",
            name="scene_renderer",
            output="screen",
            parameters=[{
                "camera_width": 640,
                "camera_height": 480,
                "camera_fps": 30,
                "display_enabled": True,
                "display_window_name": "object_rendering",
                "task_name": "grasping_in_clutter",
                "scene_file": "0.npz",
                "renderer": "tiny",
                "overlay_alpha": 0.6,
                "tag_family": "tag36h11",
                "tag_size": 0.12,
            }],
        ),
    ])
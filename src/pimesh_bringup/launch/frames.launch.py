"""
The static half of the TF tree.

REP-105 says the chain is map -> odom -> base_link, and REP-103 fixes the
conventions each frame follows. Only the static edges live here; the two
estimated edges are published by the nodes that estimate them, and **exactly
one publisher owns each edge** — two nodes publishing odom -> base_link is the
failure that smears a mesh and shows up in no single log.

    map ──(pose graph correction; identity until a backend exists)──▶ odom
    odom ──(visual odometry, keypoint_node)──▶ base_link
    base_link ──(static, here)──▶ camera_link
    camera_link ──(static, here)──▶ camera_optical_frame

base_link -> camera_link is identity today because the camera *is* the robot:
there is no body to offset from. It exists anyway so that the day a mount or a
pan-tilt appears, one number changes here instead of every node's geometry.

camera_link -> camera_optical_frame is the standard body-to-optical rotation:
body is x-forward / y-left / z-up, optical is z-forward / x-right / y-down, so
the rotation is roll -90 deg, yaw -90 deg. All geometry in this project —
keypoint rays, depth unprojection, TSDF integration — happens in the optical
frame, and this is the transform that gets it back into the world.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HALF_PI = 1.5707963267948966


def generate_launch_description():
    publish_map = LaunchConfiguration('publish_map_identity')

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_map_identity', default_value='true',
            description='publish map -> odom as identity so RViz has a fixed '
                        'frame before a pose-graph backend exists. Set false '
                        'the moment something real owns that edge — one '
                        'publisher per edge, never two'),

        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='map_to_odom_identity',
            condition=IfCondition(publish_map),
            arguments=[
                '--frame-id', 'map', '--child-frame-id', 'odom',
            ]),

        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_camera',
            arguments=[
                '--frame-id', 'base_link', '--child-frame-id', 'camera_link',
            ]),

        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='camera_to_optical',
            arguments=[
                '--roll', str(-HALF_PI), '--pitch', '0.0', '--yaw', str(-HALF_PI),
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_optical_frame',
            ]),
    ])

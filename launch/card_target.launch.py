# Copyright 2026 Team Jubilant
# SPDX-License-Identifier: Apache 2.0
# ArUco カードから押印欄を求めて target_0 を出すノードと、手先に付けたカメラの取り付け位置（static TF）を起動する。
#
# カメラの取り付け位置（mount_*）は、手先（crane_x7_gripper_base_link）からカメラの光学座標系
# （camera_frame。z が前、x が右、y が下）までのずれ。マウントが決まったら測って渡す。
# 例: ros2 launch crane_x7_stamp card_target.launch.py mount_x:=0.03 mount_z:=0.05 stamp_point_mm:="[180.0, 110.0]"

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera/color/camera_info'),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame',
                              description='カメラの画像の header.frame_id'),
        DeclareLaunchArgument('mount_parent', default_value='crane_x7_gripper_base_link',
                              description='カメラを取り付けたリンク'),
        DeclareLaunchArgument('publish_mount', default_value='true',
                              description='取り付け位置の static TF を出すか（別で出すなら false）'),
        DeclareLaunchArgument('stamp_point_mm', default_value='[0.0, 0.0]',
                              description='押印欄の位置（台紙の向き、紙の左上から mm）。[0.0, 0.0] なら Claude で探す'),
    ]
    mount = {}
    for name in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        args.append(DeclareLaunchArgument(f'mount_{name}', default_value='0.0'))
        mount[name] = LaunchConfiguration(f'mount_{name}')

    mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_tf',
        arguments=['--x', mount['x'], '--y', mount['y'], '--z', mount['z'],
                   '--roll', mount['roll'], '--pitch', mount['pitch'], '--yaw', mount['yaw'],
                   '--frame-id', LaunchConfiguration('mount_parent'),
                   '--child-frame-id', LaunchConfiguration('camera_frame')],
        condition=IfCondition(
            LaunchConfiguration('publish_mount')),
    )
    card_node = Node(
        package='crane_x7_stamp',
        executable='card_target_publisher.py',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'stamp_point_mm': ParameterValue(LaunchConfiguration('stamp_point_mm'), value_type=None),
        }],
    )
    return LaunchDescription(args + [
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        mount_tf,
        card_node,
    ])

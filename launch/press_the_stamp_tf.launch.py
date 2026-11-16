# Copyright 2023 Keitaro Nakamura
# Copyright 2024 Akira Matsumoto
# SPDX-License-Identifier: Apache 2.0
# このファイルはKeitaro Nakamuraによって作成されたcamera_picking.launch.pyをコピーし, 一部を改変したものです.
# 変更内容
#   color_detection : fixed_position_publisher
#   crane_x7_simple_example : crane_x7_stamp
#   pick_and_move_tf : press_the_stamp_tf

from crane_x7_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


# Jazzy 移植: crane_x7_examples (jazzy) と同じく MoveItConfigsBuilder で MoveIt の設定を渡す
def generate_launch_description():
    description_loader = RobotDescriptionLoader()
    moveit_config = MoveItConfigsBuilder('crane_x7').to_moveit_configs()
    moveit_config.robot_description = {
        'robot_description': description_loader.load(),
    }

    # Gazebo用
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )
    # アームの動きのコードを起動するのはこっち
    picking_node = Node(# name="pick_and_move_tf",
                        package='crane_x7_stamp',
                        executable='press_the_stamp_tf',
                        output='screen',
                        parameters=[moveit_config.to_dict()])
    # 固定座標を送るコードを起動するのはこっち
    detection_node = Node(# name='fixed_position_publisher'
                          package='crane_x7_stamp',
                          executable='fixed_position_publisher',
                          output='screen')

    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        picking_node,
        detection_node,
    ])

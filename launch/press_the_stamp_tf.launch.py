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
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
    # 2026 Jubilant: カードの押印欄（card_target.launch.py）を押すときは false にして、固定座標を出さない
    declare_use_fixed_target = DeclareLaunchArgument(
        'use_fixed_target', default_value='true',
        description='fixed_position_publisher で固定座標の target_0 を出すか')

    # 押し込みの寸法（m）。はんこを測ったら、ここか起動時の引数（例 stamp_below_grip:=0.05）で合わせる
    press_args = {
        'grip_offset': ('0.07', 'gripper_base_link から、はんこを掴む位置までの距離'),
        'stamp_below_grip': ('0.04', '掴む位置から印面までの高さ'),
        'paper_z': ('0.0', '紙の表面の高さ（base_link 基準）'),
        'press_depth': ('0.003', '紙の表面よりどれだけ下まで押し込むか'),
    }
    declare_press_args = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, (default, desc) in press_args.items()
    ]
    press_params = {
        name: ParameterValue(LaunchConfiguration(name), value_type=float) for name in press_args
    }

    # アームの動きのコードを起動するのはこっち
    picking_node = Node(# name="pick_and_move_tf",
                        package='crane_x7_stamp',
                        executable='press_the_stamp_tf',
                        output='screen',
                        parameters=[moveit_config.to_dict(), press_params])
    # 固定座標を送るコードを起動するのはこっち
    detection_node = Node(# name='fixed_position_publisher'
                          package='crane_x7_stamp',
                          executable='fixed_position_publisher',
                          output='screen',
                          condition=IfCondition(LaunchConfiguration('use_fixed_target')))

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_fixed_target,
        *declare_press_args,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        picking_node,
        detection_node,
    ])

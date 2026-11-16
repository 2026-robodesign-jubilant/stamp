# SPDX-FileCopyrightText: 2023 Keitaro Nakamura
# SPDX-License-Identifier: Apache 2.0
# このファイルはKeitaro Nakamuraによって作成されたpick_and_move.launch.pyをコピーし, Akira Matsumotoが一部を改変したものです.
# 変更内容　動作するパッケージ名とプログラム名の変更
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
    #パッケージ名の変更、実行するコードの変更は以下を編集してください
    example_node = Node(
                        package='crane_x7_stamp',
                        executable='press_the_stamp',
                        output='screen',
                        parameters=[moveit_config.to_dict()])

    return LaunchDescription([
        declare_use_sim_time,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        example_node
    ])

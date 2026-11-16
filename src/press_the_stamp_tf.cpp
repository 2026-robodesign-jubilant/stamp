// Copyright 2024 Keitaro Nakamura
// SPDX-FileCopyrightText: 2024 Haruto Yamamoto, Akira Matsumoto
// SPDX-License-Identifier: Apache 2.0
// このファイルは元々Keitaro NakamuraとRyotaro Karikomiによって作成され, その後Haruto YamamotoとAkira Matsumoyoによって変更されました.
// 変更内容　ホームポジションの角度を調整, ピッキング動作をスタンピング動作に改変.
#include <chrono>
#include <cmath>
#include <memory>
#include <vector>
#include <iostream> // std::cout を使うために必要

#include "angles/angles.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// ノード定義
class PickAndPlaceTf : public rclcpp::Node
{
public:
  // コンストラクタ
  PickAndPlaceTf(
    rclcpp::Node::SharedPtr move_group_arm_node,
    rclcpp::Node::SharedPtr move_group_gripper_node)
  : Node("pick_and_move_tf_node")
  {
    using namespace std::placeholders;

    // アームの設定
    move_group_arm_ = std::make_shared<MoveGroupInterface>(move_group_arm_node, "arm");
    move_group_arm_->setMaxVelocityScalingFactor(0.6);  // アームの最大速度スケーリング
    move_group_arm_->setMaxAccelerationScalingFactor(0.6);  // アームの最大加速度スケーリング

    // グリッパーの設定
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(move_group_gripper_node, "gripper");
    move_group_gripper_->setMaxVelocityScalingFactor(1.0);  // グリッパーの最大速度スケーリング
    move_group_gripper_->setMaxAccelerationScalingFactor(1.0);  // グリッパーの最大加速度スケーリング

    // ホームポジションに設定
    // ホームポジションへの移動は、関節角を受け取れるようになってから on_timer() の最初で行う

    // 押し込みのパラメータ（単位は m・秒）。launch の引数や --ros-args -p で変えられる
    // 2026 Jubilant: はんこの寸法を測ったら stamp_below_grip を合わせる
    grip_offset_ = this->declare_parameter("grip_offset", 0.07);  // gripper_base_link から、はんこを掴む位置（指の中ほど）までの距離（手先の向きに沿って）
    stamp_below_grip_ = this->declare_parameter("stamp_below_grip", 0.04);  // 掴む位置から印面までの高さ（はんこを縦に持つ前提）
    paper_z_ = this->declare_parameter("paper_z", 0.0);  // 紙の表面の高さ（base_link 基準）
    hover_height_ = this->declare_parameter("hover_height", 0.05);  // 押す前に印面を止めておく、紙からの高さ
    press_depth_ = this->declare_parameter("press_depth", 0.003);  // 紙の表面よりどれだけ下まで押し込むか
    press_hold_sec_ = this->declare_parameter("press_hold_sec", 1.0);  // 押し込んだまま止める時間
    press_speed_scale_ = this->declare_parameter("press_speed_scale", 0.1);  // 押し込み・持ち上げの速さ（最大速度に対する割合）

    // TFバッファとリスナーの初期化
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // タイマーの設定
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PickAndPlaceTf::on_timer, this));  // タイマーをセットし定期的にon_timerを呼び出す
  }

private:
  // タイマー処理
  void on_timer()
  {
    // ホームポジションに設定（最初の 1 回だけ）。
    // 2026 Jubilant: コンストラクタの中ではまだ関節角を受け取れず、出発点を範囲内に収められないのでここに移した
    if (!homed_) {
      move_group_arm_->setNamedTarget("home");
      plan_and_execute_arm();
      homed_ = true;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_->lookupTransform("base_link", "target_0", tf2::TimePointZero);  // "base_link"から"target_0"への変換を取得
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform base_link to target: %s", ex.what());  // 変換失敗時にログ出力
      return;
    }

    rclcpp::Time now = this->get_clock()->now();
    const std::chrono::nanoseconds FILTERING_TIME = 2s;  // フィルタリング時間
    const std::chrono::nanoseconds STOP_TIME_THRESHOLD = 3s;  // 停止時間しきい値
    const double DISTANCE_THRESHOLD = 0.01;  // 距離しきい値
    tf2::Stamped<tf2::Transform> tf;
    tf2::convert(tf_msg, tf);  // tf変換
    const auto TF_ELAPSED_TIME = now.nanoseconds() - tf.stamp_.time_since_epoch().count();  // 時間差
    const auto TF_STOP_TIME = now.nanoseconds() - tf_past_.stamp_.time_since_epoch().count();  // 停止時間
    const double TARGET_Z_MIN_LIMIT = 0.04;  // Z軸最小限界

    // フィルタリングと停止判定
    if (TF_ELAPSED_TIME < FILTERING_TIME.count()) {
      double tf_diff = (tf_past_.getOrigin() - tf.getOrigin()).length();  // 位置差
      if (tf_diff < DISTANCE_THRESHOLD) {
        if (TF_STOP_TIME > STOP_TIME_THRESHOLD.count()) {
          if (tf.getOrigin().z() < TARGET_Z_MIN_LIMIT) {
            tf.getOrigin().setZ(TARGET_Z_MIN_LIMIT);  // Z位置制限
          }
          stamping(tf.getOrigin());  // スタンピング（動作）開始
        }
      } else {
        tf_past_ = tf;  // 過去の変換を更新
      }
    }
  }

  // 今の関節角を「動ける範囲」に収めたものを、計画の出発点にする。
  // 2026 Jubilant: 紙の真上で低く構えると、肩の関節が範囲の端（約 -92°）をわずかに越え、
  // Jazzy の MoveIt は「出発点が範囲外」として次の計画を止める（CheckStartStateBounds）。その対策。
  void set_start_state_in_bounds()
  {
    auto state = move_group_arm_->getCurrentState(1.0);
    if (!state) {
      move_group_arm_->setStartStateToCurrentState();
      return;
    }
    state->enforceBounds();
    move_group_arm_->setStartState(*state);
  }

  // 範囲内に収めた出発点から計画して、実行する。
  // move() は出発点の指定を無視して今の関節角から計画するので、plan() と execute() に分けている。
  bool plan_and_execute_arm()
  {
    set_start_state_in_bounds();
    MoveGroupInterface::Plan plan;
    if (move_group_arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }
    return move_group_arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  // ホームポジションの設定
  void init_pose()
  {
    std::vector<double> joint_values = {
      angles::from_degrees(0.0),
      angles::from_degrees(85),
      angles::from_degrees(0.0),
      angles::from_degrees(-154),
      angles::from_degrees(0.0),
      angles::from_degrees(-50),
      angles::from_degrees(90)
    };
    move_group_arm_->setJointValueTarget(joint_values);  // ジョイント値を設定
    plan_and_execute_arm();  // アームを移動
  }

// 特定の関節を指定して動かす
void move_specific_joint_step(int joint_index, double relative_angle_deg)
{
  // 現在の関節値を取得
  std::vector<double> current_joint_values = move_group_arm_->getCurrentJointValues();

  // 有効な関節インデックスかを確認
  if (joint_index >= 0 && joint_index < current_joint_values.size()) {
    
    // 現在の角度を度単位で取得
    double current_angle_deg = angles::to_degrees(current_joint_values[joint_index]);

    // 角度の差分を計算
    double angle_difference = relative_angle_deg;

    // 新しい関節角度を計算
    current_joint_values[joint_index] = angles::from_degrees(current_angle_deg + angle_difference);

    // 設定したジョイント値をターゲットとして指定
    move_group_arm_->setJointValueTarget(current_joint_values);

    // 移動を実行
    move_group_arm_->move();
  }
  else {
    // 無効な関節インデックスの場合
    std::cerr << "Invalid joint index: " << joint_index << std::endl;
  }
}

  // スタンピング動作
  // 2026 Jubilant: P班の版は紙の上で止まってホームに戻っていた（押し込みはコメントアウト）。
  // 印面の真上まで動かし、そこから真下にまっすぐ押し込んで、少し止めてから真上に戻す動きを足した。
  void stamping(tf2::Vector3 target_position)
  {
    const double GRIPPER_DEFAULT = 0.0;  // デフォルトのグリッパー角度
    const int move_steps = 4;  // 移動ステップ数
    const double ROLL = 90, PITCH = 0, YAW = 90;  // 手先の向き（P班と同じ。指が水平に前を向き、はんこは縦に持つ）

    control_gripper(GRIPPER_DEFAULT);  // グリッパーをデフォルトに設定
    init_pose();  // ホームポジションに移動

    // 印面を (x, y, z) に置きたいとき、gripper_base_link をどこに置けばよいか。
    // 掴む位置は gripper_base_link から手先の向き（z 軸）に grip_offset 先、印面はそこから stamp_below_grip 下。
    tf2::Quaternion q;
    q.setRPY(angles::from_degrees(ROLL), angles::from_degrees(PITCH), angles::from_degrees(YAW));
    const tf2::Vector3 approach = tf2::Matrix3x3(q) * tf2::Vector3(0, 0, 1);  // 手先の向き（base_link 基準）
    auto gripper_pose_for_face = [&](double face_z) {
        tf2::Vector3 grip(target_position.x(), target_position.y(), face_z + stamp_below_grip_);
        tf2::Vector3 base = grip - approach * grip_offset_;
        geometry_msgs::msg::Pose pose;
        pose.position.x = base.x();
        pose.position.y = base.y();
        pose.position.z = base.z();
        pose.orientation = tf2::toMsg(q);
        return pose;
      };
    const auto hover_pose = gripper_pose_for_face(paper_z_ + hover_height_);  // 印面が紙の hover_height 上
    const auto press_pose = gripper_pose_for_face(paper_z_ - press_depth_);   // 印面が紙の press_depth 下

    // 印面の真上（hover_pose）まで、P班と同じく途中の点を経由して近づく
    geometry_msgs::msg::Pose current_pose = move_group_arm_->getCurrentPose().pose;
    for (int i = 1; i <= move_steps; ++i) {
      const double r = static_cast<double>(i) / move_steps;
      const double x = current_pose.position.x + (hover_pose.position.x - current_pose.position.x) * r;
      const double y = current_pose.position.y + (hover_pose.position.y - current_pose.position.y) * r;
      const double z = current_pose.position.z + (hover_pose.position.z - current_pose.position.z) * r;
      if (!control_arm(x, y, z, ROLL, PITCH, YAW)) {
        RCLCPP_ERROR(this->get_logger(), "印面の真上まで動けませんでした（%d/%d）。押さずに戻ります", i, move_steps);
        finish();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "近づく %d/%d", i, move_steps);
    }

    // 真下にまっすぐ押し込む → 止める → 真上に戻す
    if (move_straight(press_pose, "押し込み")) {
      RCLCPP_INFO(this->get_logger(), "押し込みました（%.1f 秒止めます）", press_hold_sec_);
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(press_hold_sec_ * 1000)));
      move_straight(hover_pose, "持ち上げ");
    }
    finish();
  }

  // 手先の向きを変えずに、今の位置から pose までまっすぐ（直線で）ゆっくり動かす
  bool move_straight(const geometry_msgs::msg::Pose & pose, const char * label)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    set_start_state_in_bounds();
    move_group_arm_->setMaxVelocityScalingFactor(press_speed_scale_);
    move_group_arm_->setMaxAccelerationScalingFactor(press_speed_scale_);
    const double fraction = move_group_arm_->computeCartesianPath({pose}, 0.005, trajectory);
    bool ok = false;
    if (fraction < 0.99) {
      // 途中までしか直線で動けない（届かない・関節の限界）ときは、中途半端に押さない
      RCLCPP_ERROR(this->get_logger(), "%s：直線で動ける割合が %.0f%% しかないので中止します", label, fraction * 100);
    } else {
      ok = move_group_arm_->execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
      if (!ok) {
        RCLCPP_ERROR(this->get_logger(), "%s：実行に失敗しました", label);
      }
    }
    move_group_arm_->setMaxVelocityScalingFactor(0.6);  // コンストラクタと同じ値に戻す
    move_group_arm_->setMaxAccelerationScalingFactor(0.6);
    return ok;
  }

  // ホームポジションに戻ってノードを終える
  void finish()
  {
    init_pose();
    rclcpp::shutdown();
  }

  // グリッパー制御
  void control_gripper(const double angle)
  {
    auto joint_values = move_group_gripper_->getCurrentJointValues();  // 現在のグリッパーの関節値を取得
    joint_values[0] = angle;  // グリッパーの角度を設定
    move_group_gripper_->setJointValueTarget(joint_values);  // ジョイント値ターゲットを設定
    move_group_gripper_->move();  // グリッパーを移動
  }

  // アーム制御
  bool control_arm(
    const double x, const double y, const double z,
    const double roll, const double pitch, const double yaw)
  {
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion q;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    q.setRPY(angles::from_degrees(roll), angles::from_degrees(pitch), angles::from_degrees(yaw));  // 回転角度を設定
    target_pose.orientation = tf2::toMsg(q);  // クォータニオンに変換
    move_group_arm_->setPoseTarget(target_pose);  // 目標位置を設定
    return plan_and_execute_arm();  // アームを移動
  }

  // メンバ変数
  std::shared_ptr<MoveGroupInterface> move_group_arm_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  tf2::Stamped<tf2::Transform> tf_past_;
  double grip_offset_;
  double stamp_below_grip_;
  double paper_z_;
  double hover_height_;
  double press_depth_;
  double press_hold_sec_;
  double press_speed_scale_;
  bool homed_ = false;
};

int main(int argc, char **argv)
{
  // rclcppの初期化
  rclcpp::init(argc, argv);

  // ノードオプションの設定
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  // ノードの作成
  auto move_group_arm_node = rclcpp::Node::make_shared("move_group_arm_node", node_options);
  auto move_group_gripper_node = rclcpp::Node::make_shared("move_group_gripper_node", node_options);

  // マルチスレッドエグゼキュータの設定
  rclcpp::executors::MultiThreadedExecutor exec;

  // PickAndPlaceTfノードの作成
  auto pick_and_move_tf_node = std::make_shared<PickAndPlaceTf>(
    move_group_arm_node,
    move_group_gripper_node);

  // ノードをエグゼキュータに追加
  exec.add_node(pick_and_move_tf_node);
  exec.add_node(move_group_arm_node);
  exec.add_node(move_group_gripper_node);

  // エグゼキュータの実行
  exec.spin();

  // rclcppのシャットダウン
  rclcpp::shutdown();

  return 0;
}


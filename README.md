# crane_x7_stamp（ROS 2 Jazzy 版）

ロボット設計製作論 2026 チーム Jubilant が、2024 年度 P班の [ken222d/crane_x7_stamp](https://github.com/ken222d/crane_x7_stamp)（Apache License 2.0）を ROS 2 Jazzy で動くように移植したものです。元の README は下にそのまま残しています。

## Jazzy 移植で変えたこと

- C++ の版を 14 から 17 に変更（Jazzy の MoveIt 2 が必要とするため）
- `CMakeLists.txt` に `angles` と `tf2_ros` の `find_package` を追加、`package.xml` に `tf2_ros` を追加
- 使っていない `realsense2_camera` の依存を削除
- launch ファイル 3 本を、rt-net の `crane_x7_examples`（jazzy）と同じく `MoveItConfigsBuilder` で MoveIt の設定を渡す形に変更

## 動作環境

- Ubuntu 24.04 / ROS 2 Jazzy
- [rt-net/crane_x7_ros](https://github.com/rt-net/crane_x7_ros) と [rt-net/crane_x7_description](https://github.com/rt-net/crane_x7_description) の `jazzy` ブランチ

```bash
mkdir -p ~/ros2/stamp_ws/src && cd ~/ros2/stamp_ws/src
git clone -b jazzy https://github.com/rt-net/crane_x7_ros.git
git clone -b jazzy https://github.com/rt-net/crane_x7_description.git
git clone https://github.com/2026-robodesign-jubilant/stamp.git crane_x7_stamp
cd ~/ros2/stamp_ws
source /opt/ros/jazzy/setup.bash
rosdep install -r -y -i --from-paths src --rosdistro jazzy
colcon build --symlink-install
```

## Gazebo で固定座標に押しに行く

端末 1:

```bash
cd ~/ros2/stamp_ws && source install/setup.bash
export ROS_DOMAIN_ID=91 GZ_PARTITION=stamp_sim
ros2 launch crane_x7_gazebo crane_x7_with_table.launch.py
```

端末 1 に `You can start planning now!` が出てから、端末 2:

```bash
cd ~/ros2/stamp_ws && source install/setup.bash
export ROS_DOMAIN_ID=91 GZ_PARTITION=stamp_sim
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py use_sim_time:=true
```

2026-11-16 に確認した結果：印面の真上まで近づき、真下にまっすぐ押し込んで 1 秒止め、真上に戻してからホームに戻ります（Gazebo で 17 回の動きがすべて成功）。`fixed_position_publisher` は止めるまで座標を送り続けるので、終わったら Ctrl+C で止めてください。

## 押し込みの寸法を合わせる

`press_the_stamp_tf` は、はんこの寸法を次のパラメータで受け取ります（単位は m）。起動時に `名前:=値` で変えられます。

| 名前 | 既定値 | 意味 |
|---|---|---|
| `grip_offset` | 0.07 | `crane_x7_gripper_base_link` から、はんこを掴む位置（指の中ほど）までの距離 |
| `stamp_below_grip` | 0.04 | 掴む位置から印面までの高さ（はんこを縦に持つ前提） |
| `paper_z` | 0.0 | 紙の表面の高さ（`base_link` 基準） |
| `press_depth` | 0.003 | 紙の表面よりどれだけ下まで押し込むか |

```bash
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py stamp_below_grip:=0.05 press_depth:=0.002
```

**実機で初めて動かすときは、`paper_z:=0.05` のように紙より高い位置を指定して、空中で押す動きを確かめてから**本当の高さに下げてください。

## ArUco カードの押印欄を押す（card_target_publisher）

`card_target_publisher.py` は、四隅に ArUco マーカー（ID 1〜4）を印刷したカード（ocr リポジトリの `test_docs/card/corner0.pdf`）をカメラで見つけ、押印欄の位置を TF の `target_0` として出します。`press_the_stamp_tf` はそれを読んで押しに行きます。

1. 4 つのマーカーの角（16 点）とカードの寸法から、カメラに対するカードの位置・向きを求めて TF `カメラ → card` を出す
2. カードが 1 秒止まったら、真上から見た画像（読む向き、594 × 840 px）を Claude（`claude-opus-5-5`）に 1 回だけ見せて押印欄を探させる
3. 見つかったら TF `card → target_0` も出す。カードが 3 秒見えなくなったら押印欄を忘れる（次のカードでまた探す）

`card` の座標系は、台紙の向きで紙の左上が原点、x が右、y が下（単位 m）です。

```bash
# カメラの取り付け位置（手先 crane_x7_gripper_base_link → カメラの光学座標系）は、マウントが決まったら mount_x などで渡す
# Claude を使う（ANTHROPIC_API_KEY と、anthropic・pydantic>=2・brotli>=1.2 が入った仮想環境が要る）
ros2 launch crane_x7_stamp card_target.launch.py mount_x:=0.03 mount_z:=0.05
# Claude を使わずに試す：押印欄の位置を台紙の左上からの mm で渡す
ros2 launch crane_x7_stamp card_target.launch.py stamp_point_mm:="[135.0, 118.0]"
# 押す側は固定座標を出さずに起動する
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py use_fixed_target:=false
```

画像は `/camera/camera/color/image_raw`、カメラの情報（焦点距離など）は `/camera/camera/color/camera_info` を読みます。別のカメラに替えるときは `image_topic`・`camera_info_topic`・`camera_frame` を変えてください。Web カメラなど `camera_info` を出さないカメラは、先にキャリブレーションが必要です。

2026-11-16 に合成画像（カメラから 47 cm、斜め・回転あり）で確認した結果：`target_0` の位置は真の値と 1 mm 以内で一致。Claude で探す部分は未確認です。

---

# crane_x7_stamp
ロボット設計製作論3 P班  
指定された座標にハンコを押すためのパッケージです.

# 環境
- Ubuntu 22.04
- ROS 2 Humble

# セットアップ, ビルド

ROS 2及びCRANE-X7のセットアップ, ビルドの方法に関しては[https://github.com/cit22ros2/crane_x7_simple_examples](https://github.com/cit22ros2/crane_x7_simple_examples)の以下をを確認してください.
- このパッケージを使う前に
  - ROS 2及びCRANE-X7セットアップ
- このパッケージの使い方
  - インストール
  - ビルド

# このパッケージの使い方

## 実行

### 推奨実行手順
1. open_close_handでハンドを開き, ハンコを掴ませる.  
2. press_the_stampもしくはpress_the_stamp_tfでハンコを押す.

### open_close_hand
ハンコを掴ませるためにハンドを5秒間開きます.
- 実機用
```
ros2 run crane_x7_stamp open_close_hand
```
- Gazebo用
```
ros2 run crane_x7_stamp open_close_hand use_sim_time:='true'
```

### press_the_stamp
ノード内で指定した座標にハンコを押します.
- 実機用
```
ros2 launch crane_x7_stamp press_the_stamp.launch.py
```
- Gazebo用
```
ros2 launch crane_x7_stamp press_the_stamp.launch.py use_sim_time:='true'
```

### press_the_stamp_tf
[![YouTubeの動画](https://img.youtube.com/vi/GrYh_InDjL0/0.jpg)](https://youtu.be/GrYh_InDjL0)  
他のノード(fixed_position_publisher)から流れてきた座標の情報をもとにハンコを押します.
- 実機用
```
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py
```

- Gazebo用
```
ros2 launch crane_x7_stamp press_the_stamp_tf.launch.py use_sim_time:='true'
```

# 注意
- 動作が大きくなることがあるので, 使用時は周囲に十分注意してください.
- マニピュレータ動作終了後, ```Ctrl+C```でプログラムを終了してください.

# 引き継ぎ
## このパッケージに変更を加えたい場合
- 新たな実行ファイルを追加する場合, [CMakeLists.txt](https://github.com/ken222d/crane_x7_stamp/blob/main/CMakeLists.txt)を編集してください.
## srcファイルの説明
- fixed_position_publisher.cpp
  - 座標を送信します.
- open_close_hand.cpp
  - ハンドを5秒間開き, その後閉じます.
- press_the_stamp.cpp
  - このノード(プログラム)内で指定した座標にハンコを押します.
- press_the_stamp_tf.cpp
  - 他のノード(プログラム)内で指定した座標にハンコを押します.
  - カメラで検出した座標にハンコを押したい場合などはこちらを利用してください.
## やり残したこと
- RealSenseを用いて色検出を行いハンコを押す.
- キーボードで入力した座標にハンコを押す.
- 無駄な移動経路の削減.
- YOLOでハンコを検出し掴む.

# ライセンス
- このパッケージはApache License 2.0のもとに, [crane_x7_simple_examples](https://github.com/cit22ros2/crane_x7_simple_examples)の一部を改変して利用しています.
  - 以下のファイルを改変し, 新規ファイルを作成しました.
    - color_detection.cppをもとに, fixed_position_publisher.cpp
    - pick_and_move.cppをもとに, press_the_stamp.cppおよびopen_close_hand.cpp
    - pick_and_move_tf.cppをもとに, press_the_stamp_tf.cpp
    - camera_picking.launch.pyをもとに, press_the_stamp_tf.launch.py
    - pick_and_move.launch.pyをもとに, press_the_stamp.launch.py
  - また, その他のファイルについても, 微小な変更を行いました.
- このパッケージはApache License 2.0に基づき公開されています.  
- ライセンスの全文は[LICENSE](https://github.com/ken222d/crane_x7_stamp/blob/main/LICENSE)から確認できます.

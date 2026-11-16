#!/usr/bin/env python3
# Copyright 2026 Team Jubilant
# SPDX-License-Identifier: Apache 2.0
"""ArUco カードの位置から押印欄の位置を求め、TF の target_0 として出すノード。

流れ
  1. カメラ画像から ArUco マーカー（ID 1〜4）を探す
  2. 16 個の角とカードの寸法から、カメラに対するカードの位置・向きを求める（solvePnP）
     → TF「カメラの座標系 → card」を毎フレーム出す
  3. カードが STILL_SEC 秒止まったら、真上から見た画像を作って Claude に押印欄を探させる（カード 1 枚につき 1 回）
     → それ以降は TF「card → target_0」も毎フレーム出す（カードが替わったら出すのをやめる）
  press_the_stamp_tf は base_link → target_0 を読むので、カメラの座標系が
  ロボットの TF につながっていれば（手先のカメラなら取り付け位置の static TF）、そのまま押しに行く。

カード（台紙 corner0.pdf）の決まりは ocr_recognizer の ocr_node と同じ:
  - 台紙の向きで横 210 × 縦 148.5 mm。台紙の右端が、読むときの上
  - 台紙の向きで左上 ID 1・右上 ID 2・右下 ID 3・左下 ID 4。黒い四角は一辺 30 mm、紙の端から 15.5 mm

card 座標系: 原点は台紙の向きでの紙の左上の角。x は台紙の右向き、y は台紙の下向き、z は紙の裏向き（単位 m）。

Claude を使わずに試すときは、押印欄の位置をパラメータ stamp_point_mm で直接渡す
（台紙の向きで紙の左上からの mm。例: -p stamp_point_mm:="[180.0, 110.0]"）。
"""

import base64
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

try:
    import anthropic
    from pydantic import BaseModel
except ImportError:  # stamp_point_mm を渡して Claude を使わないなら無くてよい
    anthropic = None
    BaseModel = object


# ---- カードの寸法（ocr_node と同じ） ---------------------------------------------
SHEET_W, SHEET_H = 210.0, 148.5   # 台紙の向きでの紙の大きさ（mm）
MARKER_MM = 30.0                  # 黒い四角の一辺（mm）
EDGE_MM = 15.5                    # 紙の端から黒い四角まで（mm）
MARKER_IDS = [1, 2, 3, 4]

STILL_PX = 5        # 前の画像からこれ以下しか動いていなければ「止まっている」（px）
STILL_SEC = 1.0     # 止まった状態がこの秒数続いたら押印欄を探す
LOST_SEC = 3.0      # カードがこの秒数見えなければ、別のカードになったとみなして押印欄を忘れる

# Claude に見せる画像の細かさ。1 mm = 4 px で、読む向き（A5 縦）にすると 594 × 840 px。
# 大きすぎる画像は Claude の側で縮められて座標がずれるので、長い辺を 1000 px 以下にしている
PX_PER_MM = 4
MODEL = 'claude-opus-5-5'

SYSTEM_PROMPT = """\
You look at a photo of a paper document, taken straight from above and cropped to the paper.
Find the place where a seal or stamp (hanko) should be pressed: a box or blank area labelled
for example "Seal", "Stamp", "Company seal", "Approved", "Received by", "Signature / Seal", "印", "捺印", "受領印".
If there is a printed box for it, give the centre of the box. If there are several, choose the one
the document most clearly asks the reader to stamp. If there is none, set found to false.
Coordinates are in pixels of the image you were given: x to the right, y downward, origin at the top-left.
"""


class StampField(BaseModel):
    found: bool          # 押印欄があったか
    center_x: float      # 押印欄の中心（画像の px、右向き）
    center_y: float      # 押印欄の中心（画像の px、下向き）
    label: str           # 押印欄の近くに書いてある言葉（例 "Received by"）
    reason: str          # そこを選んだ理由（短く）


def marker_corners_mm():
    """マーカー ID ごとの 4 つの角（台紙の向き、紙の左上からの mm）。順番は OpenCV と同じ 左上→右上→右下→左下。"""
    far_x = SHEET_W - EDGE_MM - MARKER_MM
    far_y = SHEET_H - EDGE_MM - MARKER_MM
    tl = {1: (EDGE_MM, EDGE_MM), 2: (far_x, EDGE_MM), 3: (far_x, far_y), 4: (EDGE_MM, far_y)}
    s = MARKER_MM
    return {i: np.array([[x, y], [x + s, y], [x + s, y + s], [x, y + s]], np.float32)
            for i, (x, y) in tl.items()}


def rotation_matrix_to_quaternion(r):
    """3×3 の回転行列を (x, y, z, w) の四元数にする。"""
    w = np.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = np.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2
    y = np.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2
    z = np.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2
    x = np.copysign(x, r[2, 1] - r[1, 2])
    y = np.copysign(y, r[0, 2] - r[2, 0])
    z = np.copysign(z, r[1, 0] - r[0, 1])
    return x, y, z, w


class CardTargetPublisher(Node):

    def __init__(self):
        super().__init__('card_target_publisher')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('card_frame', 'card')
        self.declare_parameter('target_frame', 'target_0')
        self.declare_parameter('stamp_point_mm', [0.0, 0.0])   # [0, 0] のときは Claude で探す

        self.card_frame = self.get_parameter('card_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        manual = list(self.get_parameter('stamp_point_mm').value)
        self.manual_point = manual if any(v != 0.0 for v in manual) else None

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        ref = marker_corners_mm()
        self.ref_mm = ref
        # solvePnP 用の 3 次元の点（card 座標系、m）。ID 1〜4 の順に 4 角ずつ
        self.object_points = np.concatenate(
            [np.hstack([ref[i] / 1000.0, np.zeros((4, 1), np.float32)]) for i in MARKER_IDS])

        self.camera_matrix = None
        self.dist_coeffs = None
        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_pts = None
        self.still_since = None
        self.last_seen = None
        self.field_mm = None        # 見つけた押印欄（台紙の向き、mm）
        self.searching = False      # Claude に問い合わせ中
        self.client = None
        if self.manual_point is None:
            if anthropic is None:
                raise RuntimeError('anthropic が入っていません。stamp_point_mm を渡すか、仮想環境を有効にしてください')
            self.client = anthropic.Anthropic()

        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self.on_camera_info, 10)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.on_image, 10)
        mode = f'手動で {self.manual_point} mm' if self.manual_point else 'Claude で探す'
        self.get_logger().info(f'カードを待っています（押印欄: {mode}）')

    def on_camera_info(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info(f'カメラの情報を受け取りました（座標系 {msg.header.frame_id}）')
        self.camera_matrix = np.array(msg.k, np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, np.float64)

    def on_image(self, msg):
        if self.camera_matrix is None:
            return   # カメラの情報（焦点距離など）が来るまでは位置を計算できない
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
        now = time.monotonic()

        if ids is None or sorted(set(int(i) for i in ids.flatten()) & set(MARKER_IDS)) != MARKER_IDS:
            self.prev_pts = None
            self.still_since = None
            if self.field_mm is not None and self.last_seen and now - self.last_seen > LOST_SEC:
                self.get_logger().info('カードが見えなくなったので、押印欄を忘れます')
                self.field_mm = None
            return
        self.last_seen = now

        # ID 1〜4 の順に、4 角ずつ並べる（同じ ID が 2 つ見えたら最初の 1 つ）
        by_id = {}
        for c, i in zip(corners, ids.flatten()):
            by_id.setdefault(int(i), c.reshape(4, 2))
        image_points = np.concatenate([by_id[i] for i in MARKER_IDS]).astype(np.float64)

        # --- カードの位置と向き（カメラの座標系） ---
        ok, rvec, tvec = cv2.solvePnP(self.object_points, image_points,
                                      self.camera_matrix, self.dist_coeffs,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            return
        self.publish_card(msg.header, rvec, tvec)

        # --- 止まったかどうか ---
        if self.prev_pts is not None and np.abs(image_points - self.prev_pts).max() <= STILL_PX:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None
        self.prev_pts = image_points

        if self.field_mm is not None or self.searching:
            return
        if self.still_since is None or now - self.still_since < STILL_SEC:
            return

        if self.manual_point is not None:
            self.set_field(self.manual_point, '手動で指定')
            return
        # Claude の返事を待つ間も TF を出し続けられるよう、別スレッドで問い合わせる
        self.searching = True
        warped = self.rectify(image, image_points)
        threading.Thread(target=self.find_field_with_claude, args=(warped,), daemon=True).start()

    def publish_card(self, header, rvec, tvec):
        rot, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rot)
        t = TransformStamped()
        t.header.stamp = header.stamp
        t.header.frame_id = header.frame_id
        t.child_frame_id = self.card_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = \
            (float(v) for v in tvec.flatten())
        t.transform.rotation.x, t.transform.rotation.y = float(qx), float(qy)
        t.transform.rotation.z, t.transform.rotation.w = float(qz), float(qw)
        transforms = [t]
        if self.field_mm is not None:
            # 押印欄はカードに貼り付いているので、card からの位置は変わらない。
            # card と同じ時刻で出すことで、カードが見えている間だけ target_0 が新しくなる
            f = TransformStamped()
            f.header.stamp = header.stamp
            f.header.frame_id = self.card_frame
            f.child_frame_id = self.target_frame
            f.transform.translation.x = self.field_mm[0] / 1000.0
            f.transform.translation.y = self.field_mm[1] / 1000.0
            f.transform.rotation.w = 1.0
            transforms.append(f)
        self.tf_broadcaster.sendTransform(transforms)

    def rectify(self, image, image_points):
        """真上から見た、読む向き（台紙の右端が上）の画像にする。1 mm = PX_PER_MM px。"""
        dst = np.concatenate([self.ref_mm[i] * PX_PER_MM for i in MARKER_IDS])
        h, _ = cv2.findHomography(image_points.astype(np.float32), dst)
        size = (int(SHEET_W * PX_PER_MM), int(SHEET_H * PX_PER_MM))
        warped = cv2.warpPerspective(image, h, size)
        return cv2.rotate(warped, cv2.ROTATE_90_COUNTERCLOCKWISE)

    def find_field_with_claude(self, warped):
        try:
            ok, jpg = cv2.imencode('.jpg', warped, [cv2.IMWRITE_JPEG_QUALITY, 90])
            data = base64.standard_b64encode(jpg.tobytes()).decode('utf-8')
            h, w = warped.shape[:2]
            t0 = time.monotonic()
            # 安全上の理由で断られたときは、別のモデルが自動で答え直す（fallbacks）
            response = self.client.beta.messages.parse(
                model=MODEL,
                max_tokens=16000,
                betas=['server-side-fallback-2026-07-01'],
                fallbacks='default',
                system=SYSTEM_PROMPT,
                messages=[{
                    'role': 'user',
                    'content': [
                        {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg', 'data': data}},
                        {'type': 'text', 'text': f'The image is {w} x {h} pixels. Where should the stamp go?'},
                    ],
                }],
                output_format=StampField,
            )
            dt = time.monotonic() - t0
            if response.stop_reason == 'refusal':
                self.get_logger().warn('Claude が回答を断りました')
                return
            field = response.parsed_output
            self.get_logger().info(
                f'Claude の返事（{dt:.1f} 秒）: found={field.found} '
                f'({field.center_x:.0f}, {field.center_y:.0f}) px「{field.label}」 {field.reason}')
            if not field.found:
                self.get_logger().warn('押印欄が見つかりませんでした')
                return
            # 読む向きの画像の px → 台紙の向きの mm（rectify で左に 90° 回したのを戻す）
            x_mm = SHEET_W - field.center_y / PX_PER_MM
            y_mm = field.center_x / PX_PER_MM
            self.set_field([x_mm, y_mm], f'Claude「{field.label}」')
        except Exception as e:  # noqa: BLE001  通信の失敗などでノードを止めない
            self.get_logger().error(f'Claude への問い合わせに失敗しました: {e}')
        finally:
            self.searching = False

    def set_field(self, point_mm, how):
        x_mm, y_mm = float(point_mm[0]), float(point_mm[1])
        if not (0 <= x_mm <= SHEET_W and 0 <= y_mm <= SHEET_H):
            self.get_logger().warn(f'押印欄 ({x_mm:.1f}, {y_mm:.1f}) mm がカードの外なので使いません')
            return
        self.field_mm = (x_mm, y_mm)
        self.get_logger().info(
            f'押印欄を {self.target_frame} として出します: 台紙の左上から ({x_mm:.1f}, {y_mm:.1f}) mm（{how}）')


def main():
    rclpy.init()
    node = CardTargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2026 Team Jubilant
# SPDX-License-Identifier: Apache 2.0
"""USB Web カメラで ArUco カードを撮って、要約と押印欄の位置を確かめる画面（ROS なし）。

- 左：カメラの映像。4 つのマーカー（ID 1〜4）が見えるとカードの枠が緑になる
- 「撮影して読む」：その瞬間の画像から、真上から見た画像を作って Claude に 1 回だけ見せ、
  文書の種類・要点（日本語）と押印欄の位置を返してもらう
- 右：結果。押印欄はカード上の mm（card_target_publisher の card 座標系）と、撮った画像の px で表示し、
  撮った画像と真上から見た画像の両方に赤い点を打つ

使い方（OCR と同じ仮想環境で。anthropic・pydantic>=2・brotli>=1.2 が要る）:
    read -rs ANTHROPIC_API_KEY && export ANTHROPIC_API_KEY
    python3 card_camera_test.py                      # /dev/video2 を使う
    python3 card_camera_test.py --device /dev/video0 --expect 78,110   # 正解（mm）が分かっていれば誤差も出す
"""

import argparse
import base64
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk

import anthropic
import cv2
import numpy as np
from PIL import Image, ImageTk
from pydantic import BaseModel

# ---- カードの寸法（ocr_node・card_target_publisher と同じ） ------------------------
SHEET_W, SHEET_H = 210.0, 148.5
MARKER_MM, EDGE_MM = 30.0, 15.5
MARKER_IDS = [1, 2, 3, 4]
PX_PER_MM = 4            # Claude に見せる画像：読む向き 594 × 840 px（長い辺 1000 px 以下で座標がずれない）
MODEL = 'claude-opus-5-5'

SYSTEM_PROMPT = """\
You look at a photo of a paper document, taken straight from above and cropped to the paper.
1. Say what kind of document it is, and write a short summary in Japanese that can be read aloud
   (about 3 sentences; dates and amounts written so they are easy to hear). Do not guess text you cannot read;
   list such places in unclear.
2. Find the place where a seal or stamp (hanko) should be pressed: a box or blank area labelled for example
   "Seal", "Stamp", "Approved", "Received by", "Signature / Seal", "印", "捺印", "受領印". If there is a printed box,
   give the centre of the box. If there is none, set stamp_found to false.
   Coordinates are in pixels of the image you were given: x to the right, y downward, origin at the top-left.
"""


class CardReading(BaseModel):
    doc_type: str          # 文書の種類（日本語）
    summary: str           # 読み上げ用の要点（日本語）
    unclear: list[str]     # 読めなかった所
    stamp_found: bool      # 押印欄があったか
    stamp_x: float         # 押印欄の中心（画像の px、右向き）
    stamp_y: float         # 押印欄の中心（画像の px、下向き）
    stamp_label: str       # 押印欄の近くの言葉


def marker_corners_mm():
    far_x = SHEET_W - EDGE_MM - MARKER_MM
    far_y = SHEET_H - EDGE_MM - MARKER_MM
    tl = {1: (EDGE_MM, EDGE_MM), 2: (far_x, EDGE_MM), 3: (far_x, far_y), 4: (EDGE_MM, far_y)}
    s = MARKER_MM
    return {i: np.array([[x, y], [x + s, y], [x + s, y + s], [x, y + s]], np.float32)
            for i, (x, y) in tl.items()}


REF_MM = marker_corners_mm()
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# 台紙の向きでの紙の外形（mm）。映像に枠を描くのに使う
SHEET_OUTLINE_MM = np.float32([[0, 0], [SHEET_W, 0], [SHEET_W, SHEET_H], [0, SHEET_H]])


def find_card(frame):
    """マーカーを探し、4 つそろえば「台紙の mm → 画像の px」のホモグラフィを返す。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, DICTIONARY)
    if ids is None:
        return None, []
    found = sorted(set(int(i) for i in ids.flatten()) & set(MARKER_IDS))
    if found != MARKER_IDS:
        return None, found
    by_id = {}
    for c, i in zip(corners, ids.flatten()):
        by_id.setdefault(int(i), c.reshape(4, 2))
    img_pts = np.concatenate([by_id[i] for i in MARKER_IDS]).astype(np.float32)
    mm_pts = np.concatenate([REF_MM[i] for i in MARKER_IDS])
    h, _ = cv2.findHomography(mm_pts, img_pts)
    return h, found


def rectify(frame, h_mm_to_img):
    """真上から見た、読む向き（台紙の右端が上）の画像にする。"""
    scale = np.diag([PX_PER_MM, PX_PER_MM, 1.0])
    h_img_to_px = scale @ np.linalg.inv(h_mm_to_img)
    size = (int(SHEET_W * PX_PER_MM), int(SHEET_H * PX_PER_MM))
    warped = cv2.warpPerspective(frame, h_img_to_px, size)
    return cv2.rotate(warped, cv2.ROTATE_90_COUNTERCLOCKWISE)


def reading_px_to_card_mm(x, y):
    """読む向きの画像の px → 台紙の向きの mm（rectify で左に 90° 回したのを戻す）。"""
    return SHEET_W - y / PX_PER_MM, x / PX_PER_MM


def to_photo(bgr, max_w, max_h):
    h, w = bgr.shape[:2]
    s = min(max_w / w, max_h / h, 1.0)
    rgb = cv2.cvtColor(cv2.resize(bgr, (int(w * s), int(h * s))), cv2.COLOR_BGR2RGB)
    return ImageTk.PhotoImage(Image.fromarray(rgb))


class App:

    def __init__(self, root, device, expect):
        self.root = root
        self.expect = expect
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            raise SystemExit(f'カメラ {device} を開けません（他のソフトが使っていないか確認）')
        self.client = None   # 撮影したときに作る（キーがなくても映像は見られる）
        self.results = queue.Queue()
        self.busy = False
        self.frame = None

        root.title('カードの読み取りテスト')
        left = ttk.Frame(root, padding=6)
        left.grid(row=0, column=0, sticky='nsew')
        right = ttk.Frame(root, padding=6)
        right.grid(row=0, column=1, sticky='nsew')
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        self.video = ttk.Label(left)
        self.video.pack()
        self.live_status = ttk.Label(left, text='')
        self.live_status.pack(anchor='w')
        self.shot = ttk.Label(left)           # 撮った画像（押印欄の点つき）
        self.shot.pack(pady=(6, 0))

        self.button = ttk.Button(right, text='撮影して読む', command=self.capture)
        self.button.pack(fill='x')
        self.status = ttk.Label(right, text='カードを映して「撮影して読む」を押してください', wraplength=420)
        self.status.pack(anchor='w', pady=4)
        self.text = tk.Text(right, width=56, height=22, wrap='word', font=('Noto Sans CJK JP', 11))
        self.text.pack(fill='both', expand=True)
        self.rect_view = ttk.Label(right)     # 真上から見た画像（Claude に見せたもの）
        self.rect_view.pack(pady=(6, 0))

        root.bind('<space>', lambda e: self.capture())
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.update_video()
        self.poll_results()

    # ---- 映像 ----
    def update_video(self):
        ok, frame = self.cap.read()
        if ok:
            self.frame = frame
            view = frame.copy()
            h, found = find_card(frame)
            if h is not None:
                outline = cv2.perspectiveTransform(SHEET_OUTLINE_MM[None], h)[0].astype(np.int32)
                cv2.polylines(view, [outline], True, (0, 200, 0), 3)
                self.live_status.config(text='カードが見えています（マーカー 4 つ）')
            else:
                self.live_status.config(text=f'見えているマーカー: {found or "なし"}（ID 1〜4 が全部要る）')
            self.video_img = to_photo(view, 800, 450)
            self.video.config(image=self.video_img)
        self.root.after(40, self.update_video)

    # ---- 撮影 ----
    def capture(self):
        if self.busy or self.frame is None:
            return
        frame = self.frame.copy()
        h, found = find_card(frame)
        if h is None:
            self.status.config(text=f'マーカーが 4 つそろっていません（見えているのは {found or "なし"}）')
            return
        warped = rectify(frame, h)
        self.busy = True
        self.button.config(state='disabled')
        self.status.config(text='Claude に問い合わせ中…（10〜20 秒）')
        self.text.delete('1.0', 'end')
        self.rect_img = to_photo(warped, 300, 420)
        self.rect_view.config(image=self.rect_img)
        threading.Thread(target=self.ask_claude, args=(frame, h, warped), daemon=True).start()

    def ask_claude(self, frame, h, warped):
        try:
            if self.client is None:
                self.client = anthropic.Anthropic()
            ok, jpg = cv2.imencode('.jpg', warped, [cv2.IMWRITE_JPEG_QUALITY, 90])
            data = base64.standard_b64encode(jpg.tobytes()).decode('utf-8')
            hh, ww = warped.shape[:2]
            t0 = time.monotonic()
            response = self.client.beta.messages.parse(
                model=MODEL,
                max_tokens=16000,
                betas=['server-side-fallback-2026-07-01'],
                fallbacks='default',      # 安全上の理由で断られたら、別のモデルが答え直す
                system=SYSTEM_PROMPT,
                messages=[{'role': 'user', 'content': [
                    {'type': 'image', 'source': {'type': 'base64', 'media_type': 'image/jpeg', 'data': data}},
                    {'type': 'text', 'text': f'The image is {ww} x {hh} pixels.'},
                ]}],
                output_format=CardReading,
            )
            self.results.put(('ok', frame, h, warped, response, time.monotonic() - t0))
        except Exception as e:  # noqa: BLE001  画面に出して続ける
            self.results.put(('error', str(e)))

    def poll_results(self):
        try:
            item = self.results.get_nowait()
        except queue.Empty:
            self.root.after(100, self.poll_results)
            return
        self.busy = False
        self.button.config(state='normal')
        if item[0] == 'error':
            self.status.config(text=f'失敗しました: {item[1]}')
        else:
            self.show(*item[1:])
        self.root.after(100, self.poll_results)

    def show(self, frame, h, warped, response, dt):
        if response.stop_reason == 'refusal':
            self.status.config(text='Claude が回答を断りました')
            return
        r = response.parsed_output
        self.status.config(text=f'読み取り完了（{dt:.1f} 秒、{response.model}）')
        lines = [f'種類: {r.doc_type}', '', f'要点: {r.summary}', '']
        if r.unclear:
            lines += ['読めなかった所: ' + '、'.join(r.unclear), '']
        if r.stamp_found:
            x_mm, y_mm = reading_px_to_card_mm(r.stamp_x, r.stamp_y)
            img_pt = cv2.perspectiveTransform(np.float32([[[x_mm, y_mm]]]), h)[0, 0]
            lines += [
                f'押印欄:「{r.stamp_label}」',
                f'  真上から見た画像: ({r.stamp_x:.0f}, {r.stamp_y:.0f}) px',
                f'  カード上（台紙の向き、左上から）: ({x_mm:.1f}, {y_mm:.1f}) mm',
                f'  撮った画像: ({img_pt[0]:.0f}, {img_pt[1]:.0f}) px',
            ]
            if self.expect:
                err = np.hypot(x_mm - self.expect[0], y_mm - self.expect[1])
                lines.append(f'  正解 ({self.expect[0]:.0f}, {self.expect[1]:.0f}) mm とのずれ: {err:.1f} mm')
            shot = frame.copy()
            cv2.circle(shot, (int(img_pt[0]), int(img_pt[1])), 10, (0, 0, 255), -1)
            marked = warped.copy()
            cv2.circle(marked, (int(r.stamp_x), int(r.stamp_y)), 8, (0, 0, 255), -1)
            self.rect_img = to_photo(marked, 300, 420)
            self.rect_view.config(image=self.rect_img)
        else:
            lines.append('押印欄: 見つかりませんでした')
            shot = frame
        self.shot_img = to_photo(shot, 400, 225)
        self.shot.config(image=self.shot_img)
        self.text.insert('1.0', '\n'.join(lines))

    def close(self):
        self.cap.release()
        self.root.destroy()


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--device', default='/dev/video2', help='Web カメラ（既定 /dev/video2）')
    p.add_argument('--expect', default='', help='押印欄の正解（mm、例 78,110）。04_receipt なら 78,110')
    a = p.parse_args()
    expect = tuple(float(v) for v in a.expect.split(',')) if a.expect else None
    root = tk.Tk()
    App(root, a.device, expect)
    root.mainloop()


if __name__ == '__main__':
    main()

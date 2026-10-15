#!/usr/bin/env python3
"""カメラ画像(/camera/image_raw)から写真・動画を保存する。

写真:  python3 camera_capture.py photo -o shot.jpg
        python3 camera_capture.py photo -o shots/ --count 10 --interval 2   # 2秒ごとに10枚
動画:  python3 camera_capture.py video -o run.mp4 --duration 30             # 30秒録画
        python3 camera_capture.py video -o run.mp4                          # Ctrl+C で終了
"""
import argparse
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Capture(Node):
    def __init__(self, a):
        super().__init__('camera_capture')
        self.a = a
        self.bridge = CvBridge()
        self.writer = None
        self.saved = 0
        self.last = 0.0
        self.t0 = None
        self.done = False
        self.create_subscription(Image, a.topic, self.cb, 10)
        self.get_logger().info(f'waiting for {a.topic} ...')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        a = self.a

        if a.mode == 'photo':
            if now - self.last < a.interval and self.saved > 0:
                return
            out = Path(a.output)
            if a.count > 1:
                out.mkdir(parents=True, exist_ok=True)
                out = out / f'img_{self.saved:04d}.jpg'
            else:
                out.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(out), frame)
            self.get_logger().info(f'saved {out}')
            self.saved += 1
            self.last = now
            self.done = self.saved >= a.count
        else:
            if self.writer is None:
                Path(a.output).parent.mkdir(parents=True, exist_ok=True)
                h, w = frame.shape[:2]
                self.writer = cv2.VideoWriter(
                    a.output, cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (w, h))
                self.get_logger().info(f'recording {w}x{h} @ {a.fps} fps -> {a.output}')
            self.writer.write(frame)
            self.saved += 1
            if a.duration and now - self.t0 >= a.duration:
                self.done = True


def main():
    p = argparse.ArgumentParser()
    p.add_argument('mode', choices=['photo', 'video'])
    p.add_argument('-o', '--output', required=True)
    p.add_argument('--topic', default='/camera/image_raw')
    p.add_argument('--count', type=int, default=1, help='photo: 枚数')
    p.add_argument('--interval', type=float, default=1.0, help='photo: 間隔[s]')
    p.add_argument('--duration', type=float, default=0.0, help='video: 録画時間[s] (0=Ctrl+Cまで)')
    p.add_argument('--fps', type=float, default=30.0, help='video: 再生fps')
    a = p.parse_args()

    rclpy.init()
    node = Capture(a)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    if node.writer is not None:
        node.writer.release()
        node.get_logger().info(f'video saved: {a.output} ({node.saved} frames)')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

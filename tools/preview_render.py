#!/usr/bin/env python3
"""Gazebo を使わずに、コンテナ・ラベル・ロボットカメラからの見え方を簡易レンダリングする(レイキャスト)。

generate_world.py と同じ寸法・配置・テクスチャを使うが、Gazebo(OGRE)の描画ではない。
照明は Gazebo 既定の太陽の向き (-0.5, 0.1, -0.9) と環境光に合わせてある。
ラベルの文字は「外側から見て正立」で描くので、Gazebo 上で面ごとの UV の向きが違う場合はそこだけ差が出る。

  python3 tools/preview_render.py objects  -o preview_objects          # コンテナ単体(2アングル)
  python3 tools/preview_render.py robot    -o preview_robot.png --seed 1   # 出発位置のロボットカメラ視点
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from field_world import generate_world as g  # noqa: E402

TEX = Path(__file__).resolve().parent.parent / "models/label_materials/materials/textures"
BLUE, GREEN, LID = (0.10, 0.35, 0.80), (0.10, 0.65, 0.25), (0.14, 0.72, 0.30)
FLOOR, BORDER, GROUND = (0.85, 0.85, 0.85), (0.10, 0.10, 0.10), (0.60, 0.60, 0.60)
LIGHT = np.array([0.5, -0.1, 0.9]) / np.linalg.norm([0.5, -0.1, 0.9])   # 光源の方向(Gazebo 既定の太陽)
AMBIENT, DIFFUSE = 0.55, 0.55


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Scene:
    def __init__(self):
        self.boxes = []   # (center, R, half, color)
        self.quads = []   # (center, n, up, right, w, h, texture)

    def box(self, center, yaw, size, color):
        self.boxes.append((np.array(center, float), rotz(yaw), np.array(size, float) / 2, color))

    def quad(self, center, n, up, w, h, tex):
        n, up = np.array(n, float), np.array(up, float)
        right = np.cross(up, n)
        img = np.asarray(Image.open(TEX / tex).convert("RGB"), dtype=np.float32) / 255.0
        self.quads.append((np.array(center, float), n, up, right, w, h, img))

    # --- コンテナ(generate_world.py と同じ寸法) ---
    def delivery_box(self, x, y, yaw, letter):
        L, W, H, T = g.BOX_L, g.BOX_W, g.BOX_H, g.WALL_T
        R = rotz(yaw)
        o = np.array([x, y, 0.0])
        loc = lambda v: R @ np.array(v, float) + o  # noqa: E731
        for size, xyz in [((L, W, T), (0, 0, T / 2)),
                          ((L, T, H), (0, -(W / 2 - T / 2), H / 2)),
                          ((L, T, H), (0, W / 2 - T / 2, H / 2)),
                          ((T, W - 2 * T, H), (-(L / 2 - T / 2), 0, H / 2)),
                          ((T, W - 2 * T, H), (L / 2 - T / 2, 0, H / 2))]:
            self.box(loc(xyz), yaw, size, BLUE)
        self.quad(loc((-(L / 2 + 0.001), 0, 0.16)), R @ (-1, 0, 0), (0, 0, 1), 0.20, 0.15,
                  f"delivery_{letter}.png")

    def pickup_box(self, x, y, yaw, letter):
        L, W, H, LT = g.BOX_L, g.BOX_W, g.BOX_H, g.LID_T
        R = rotz(yaw)
        o = np.array([x, y, 0.0])
        loc = lambda v: R @ np.array(v, float) + o  # noqa: E731
        self.box(loc((0, 0, (H - LT) / 2)), yaw, (L, W, H - LT), GREEN)
        self.box(loc((0, 0, H - LT / 2)), yaw, (L + 0.01, W + 0.01, LT), LID)
        self.quad(loc((0.08, -0.04, H + 0.001)), R @ (0, 0, 1), R @ (0, -1, 0), 0.20, 0.15,
                  f"dest_{letter}.png")
        self.quad(loc((0.10, W / 2 + 0.001, 0.14)), R @ (0, 1, 0), (0, 0, 1), 0.20, 0.15,
                  f"dest_{letter}.png")
        self.quad(loc((-(L / 2 + 0.001), 0, 0.14)), R @ (-1, 0, 0), (0, 0, 1), 0.14, 0.20,
                  "pickup.png")

    def area(self, cx, cy, w, h):
        self.box((cx, cy, 0.002), 0, (w + 0.1, h + 0.1, 0.004), BORDER)
        self.box((cx, cy, 0.003), 0, (w, h, 0.006), FLOOR)


def hit_box(ro, rd, box):
    c, R, half, _ = box
    o, d = (ro - c) @ R, rd @ R
    d = np.where(np.abs(d) < 1e-9, 1e-9, d)
    t1, t2 = (-half - o) / d, (half - o) / d
    tmin, tmax = np.minimum(t1, t2), np.maximum(t1, t2)
    tn, tf = tmin.max(1), tmax.min(1)
    ok = (tn <= tf) & (tf > 1e-4)
    t = np.where(tn > 1e-4, tn, tf)
    ax = tmin.argmax(1)
    nl = np.zeros_like(o)
    nl[np.arange(len(o)), ax] = -np.sign(d[np.arange(len(o)), ax])
    return np.where(ok, t, np.inf), nl @ R.T


def hit_quad(ro, rd, quad):
    c, n, up, right, w, h, img = quad
    den = rd @ n
    t = np.where(den < -1e-9, ((c - ro) @ n) / np.where(den < -1e-9, den, -1), np.inf)
    t = np.where(t > 1e-4, t, np.inf)
    p = ro + rd * np.where(np.isfinite(t), t, 0)[:, None]
    s, v = (p - c) @ right / w + 0.5, (p - c) @ up / h + 0.5
    ok = np.isfinite(t) & (s >= 0) & (s <= 1) & (v >= 0) & (v <= 1)
    ih, iw = img.shape[:2]
    px = np.clip((s * (iw - 1)).round().astype(int), 0, iw - 1)
    py = np.clip(((1 - v) * (ih - 1)).round().astype(int), 0, ih - 1)
    return np.where(ok, t, np.inf), img[py, px]


def trace(scene, ro, rd):
    n = len(rd)
    best = np.full(n, np.inf)
    col = np.zeros((n, 3), np.float32)
    nrm = np.zeros((n, 3))
    # 地面
    with np.errstate(divide="ignore", invalid="ignore"):
        tg = np.where(rd[:, 2] < -1e-9, -ro[:, 2] / rd[:, 2], np.inf)
    m = tg < best
    best[m], nrm[m], col[m] = tg[m], (0, 0, 1), GROUND
    for b in scene.boxes:
        t, nl = hit_box(ro, rd, b)
        m = t < best
        best[m], nrm[m], col[m] = t[m], nl[m], b[3]
    # 影 (箱・床だけ遮蔽する)
    hit = np.isfinite(best)
    p = ro + rd * np.where(hit, best, 0)[:, None] + nrm * 2e-3
    lit = np.ones(n, bool)
    Lr = np.broadcast_to(LIGHT, p.shape)
    for b in scene.boxes:
        t, _ = hit_box(p, Lr, b)
        lit &= ~np.isfinite(t)
    ndl = np.clip(nrm @ LIGHT, 0, None) * lit
    shade = AMBIENT + DIFFUSE * ndl
    # ラベル(箱の面より 1mm 手前)
    qshade = shade.copy()
    for q in scene.quads:
        t, c = hit_quad(ro, rd, q)
        m = t < best + 2e-3
        best[m], col[m] = np.minimum(best[m], t[m]), c[m]
        qn = q[1]
        qshade[m] = AMBIENT + DIFFUSE * max(0.0, float(qn @ LIGHT)) * lit[m]
        hit |= m
    out = col
    shade = np.where(hit, qshade, shade)
    img = np.clip(out * shade[:, None], 0, 1)
    sky = np.clip(rd[:, 2], 0, 1)[:, None]
    skyc = (1 - sky) * np.array([0.85, 0.90, 0.97]) + sky * np.array([0.45, 0.65, 0.92])
    return np.where(hit[:, None], img, skyc)


def render(scene, eye, target, hfov_deg, w, h, ss=2):
    eye, target = np.array(eye, float), np.array(target, float)
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, (0, 0, 1))
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    W, H = w * ss, h * ss
    tx = math.tan(math.radians(hfov_deg) / 2)
    ty = tx * h / w
    xs = (np.arange(W) + 0.5) / W * 2 - 1
    ys = 1 - (np.arange(H) + 0.5) / H * 2
    out = np.zeros((H, W, 3), np.float32)
    for r0 in range(0, H, 40):
        yy = ys[r0:r0 + 40]
        gx, gy = np.meshgrid(xs, yy)
        rd = fwd + gx[..., None] * tx * right + gy[..., None] * ty * up
        rd = rd.reshape(-1, 3)
        rd /= np.linalg.norm(rd, axis=1, keepdims=True)
        ro = np.broadcast_to(eye, rd.shape)
        out[r0:r0 + 40] = trace(scene, ro, rd).reshape(len(yy), W, 3)
    img = Image.fromarray((out * 255).astype(np.uint8))
    return img.resize((w, h), Image.LANCZOS) if ss > 1 else img


def main():
    p = argparse.ArgumentParser()
    p.add_argument("view", choices=["objects", "robot"])
    p.add_argument("-o", "--output", required=True)
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--green", choices="abcd", default="b")
    a = p.parse_args()

    if a.view == "objects":
        # 比較しやすいように、緑と青を並べる(実際のワールドでは11m以上離れている)
        s = Scene()
        s.pickup_box(0.0, 0.0, 0.0, a.green)
        s.delivery_box(0.95, 0.0, -math.pi / 2, "a")  # 短辺のラベルが +y 側を向くように回転
        for k, (eye, tgt, fov) in enumerate([((0.55, 1.9, 0.95), (0.55, 0.0, 0.17), 52),
                                              ((-1.5, 1.7, 1.25), (0.45, 0.0, 0.12), 52)]):
            img = render(s, eye, tgt, fov, 1280, 720)
            img.save(f"{a.output}_{k + 1}.png")
            print("saved", f"{a.output}_{k + 1}.png")
    else:
        import random
        rng = random.Random(a.seed)
        green = a.green if a.seed is None else rng.choice("abcd")
        s = Scene()
        w, h = g.AREA_SIZE
        for label, (cx, cy) in g.AREAS.items():
            s.area(cx, cy, w, h)
            x = cx + rng.uniform(-w / 2 + g.MARGIN, w / 2 - g.MARGIN)
            y = cy + rng.uniform(-h / 2 + g.MARGIN, h / 2 - g.MARGIN)
            s.delivery_box(x, y, rng.uniform(-math.pi, math.pi), label)
        s.pickup_box(g.GREEN_POS[0], g.GREEN_POS[1], 0.0, green)
        sx, sy, syaw = g.ROBOT_START
        cam_off, cam_z = 0.40, 0.29                    # sim_nav.launch.py の camera_x / camera_z
        eye = (sx + cam_off * math.cos(syaw), sy + cam_off * math.sin(syaw), cam_z)
        tgt = (eye[0] + math.cos(syaw), eye[1] + math.sin(syaw), cam_z)
        img = render(s, eye, tgt, 70.0, 1280, 720)     # 1280x720, 水平画角 70°
        img.save(a.output)
        print("saved", a.output, "green =", green)


if __name__ == "__main__":
    main()

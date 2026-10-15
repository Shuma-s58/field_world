#!/usr/bin/env python3
"""a/b/c/d の4エリアを持つ Gazebo Classic ワールドを生成する。

- 各エリアに「宅配ボックス(青い折りたたみコンテナ・上面開放)」を1つ、エリア内ランダム位置に配置
  (短辺の側面に白いラベル「宅配ボックス + エリア名」)
- 中央に「集荷ボックス(緑の折りたたみコンテナ・蓋付き)」を1つ配置
  (上面と長辺の側面に「配達先 + a〜d(毎回ランダム)」、短辺の側面に「集荷ボックス」のラベル)
- エリア・間隔の寸法は つくばチャレンジ2026 の配置図(10mスケールバー)からの読み取り値
"""
import argparse
import math
import random
from pathlib import Path

# ---- 寸法 [m] ------------------------------------------------------------
AREA_SIZE = (13.2, 8.5)   # エリアの x, y 寸法
AREA_Y = 2.4              # エリア中心の y(緑Boxはエリアの下端から約1.9m内側にある)
GREEN_POS = (0.0, 0.0)    # 緑Boxの位置
GREEN_GAP = 11.0          # 緑Boxの端から、内側エリア(c, b)の端までの距離
AREA_GAP_CD = 1.5         # d-c 間(エリアの端から端まで)
AREA_GAP_BA = 2.1         # b-a 間

# ロボットの初期位置 (x, y, yaw): 緑Boxの上側(+y)3m から、緑Boxの方(-y 向き)
ROBOT_START = (GREEN_POS[0], GREEN_POS[1] + 3.0, -math.pi / 2)

# 折りたたみコンテナ(50L 相当の想定寸法。写真の縦横比から推定)
BOX_L, BOX_W, BOX_H = 0.53, 0.37, 0.31   # 長辺(x), 短辺(y), 高さ(z)
WALL_T = 0.015            # 青コンテナの壁厚
LID_T = 0.03              # 緑コンテナの蓋の厚み
MARGIN = 0.35             # エリア端からコンテナ中心までの最小距離(回転しても収まる値)


def _area_centers():
    # 図の左→右: d, c, (緑box), b, a
    w = AREA_SIZE[0]
    gx = GREEN_POS[0]
    inner = BOX_L / 2 + GREEN_GAP + w / 2
    return {
        "d": (gx - inner - w - AREA_GAP_CD, AREA_Y),
        "c": (gx - inner, AREA_Y),
        "b": (gx + inner, AREA_Y),
        "a": (gx + inner + w + AREA_GAP_BA, AREA_Y),
    }


AREAS = _area_centers()

MAT_URIS = (
    "<uri>model://label_materials/materials/scripts</uri>"
    "<uri>model://label_materials/materials/textures</uri>"
)


def _part(name, size, xyz, material, collide=True):
    sx, sy, sz = size
    x, y, z = xyz
    pose = f"<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>"
    geom = f"<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>"
    col = f'\n        <collision name="{name}_col">{pose}{geom}</collision>' if collide else ""
    vis = (f'\n        <visual name="{name}_vis">{pose}{geom}'
           f"<material><script>{MAT_URIS}<name>{material}</name></script></material></visual>")
    return col + vis


def _model(name, x, y, yaw, parts):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>
      <link name="link">{''.join(parts)}
      </link>
    </model>"""


def _delivery_box(name, x, y, yaw, letter):
    """青い折りたたみコンテナ(上面開放)。短辺(-x 側の端面)に白ラベル。"""
    L, W, H, T = BOX_L, BOX_W, BOX_H, WALL_T
    m = "Container/Blue"
    parts = [
        _part("bottom", (L, W, T), (0, 0, T / 2), m),
        _part("front", (L, T, H), (0, -(W / 2 - T / 2), H / 2), m),
        _part("back", (L, T, H), (0, W / 2 - T / 2, H / 2), m),
        _part("left", (T, W - 2 * T, H), (-(L / 2 - T / 2), 0, H / 2), m),
        _part("right", (T, W - 2 * T, H), (L / 2 - T / 2, 0, H / 2), m),
        _part("label", (0.002, 0.20, 0.15), (-(L / 2 + 0.001), 0, 0.16),
              f"Paper/Delivery_{letter}", collide=False),
    ]
    return _model(name, x, y, yaw, parts)


def _pickup_box(name, x, y, yaw, letter):
    """緑の折りたたみコンテナ(蓋付き)。上面・長辺側面(+y 側=ロボットの出発側)に「配達先 letter」、短辺側面に「集荷ボックス」。"""
    L, W, H = BOX_L, BOX_W, BOX_H
    parts = [
        _part("body", (L, W, H - LID_T), (0, 0, (H - LID_T) / 2), "Container/Green"),
        _part("lid", (L + 0.01, W + 0.01, LID_T), (0, 0, H - LID_T / 2), "Container/GreenLid"),
        _part("label_top", (0.20, 0.15, 0.002), (0.08, -0.04, H + 0.001),
              f"Paper/Dest_{letter}", collide=False),
        _part("label_side", (0.20, 0.002, 0.15), (0.10, W / 2 + 0.001, 0.14),
              f"Paper/Dest_{letter}", collide=False),
        _part("label_pickup", (0.002, 0.14, 0.20), (-(L / 2 + 0.001), 0, 0.14),
              "Paper/Pickup", collide=False),
    ]
    return _model(name, x, y, yaw, parts)


def _floor(name, cx, cy, w, h):
    # 外枠(濃色)の上に一回り小さい床(淡色)を重ねて境界線にする。コリジョンなし。
    def tile(n, sx, sy, z, rgb):
        return f"""
    <model name="{n}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {z / 2} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {z}</size></box></geometry>
          <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material>
        </visual>
      </link>
    </model>"""
    return (tile(f"{name}_border", w + 0.1, h + 0.1, 0.004, "0.1 0.1 0.1")
            + tile(f"{name}_floor", w, h, 0.006, "0.85 0.85 0.85"))


def generate(output, seed=None, green=None):
    rng = random.Random(seed)
    green = green or rng.choice(sorted(AREAS))
    w, h = AREA_SIZE
    placements = {}
    models = []

    for label, (cx, cy) in AREAS.items():
        models.append(_floor(f"area_{label}", cx, cy, w, h))
        x = cx + rng.uniform(-w / 2 + MARGIN, w / 2 - MARGIN)
        y = cy + rng.uniform(-h / 2 + MARGIN, h / 2 - MARGIN)
        yaw = rng.uniform(-math.pi, math.pi)
        models.append(_delivery_box(f"blue_box_{label}", x, y, yaw, label))
        placements[f"blue_box_{label}"] = (round(x, 3), round(y, 3), round(yaw, 3))

    models.append(_pickup_box("green_box", GREEN_POS[0], GREEN_POS[1], 0.0, green))

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="field">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    {''.join(models)}
  </world>
</sdf>
"""
    out = Path(output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(sdf)

    # 正解データ(評価用)
    lines = [f"seed: {seed}", f"green_label: {green}"]
    lines += [f"{k}: [{v[0]}, {v[1]}, {v[2]}]" for k, v in placements.items()]
    out.with_suffix(".answer.yaml").write_text("\n".join(lines) + "\n")
    return green, placements


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-o", "--output", default="field_random.world")
    p.add_argument("--seed", type=int, default=None, help="省略時は毎回ランダム")
    p.add_argument("--green", choices=sorted(AREAS), default=None, help="緑Boxのラベルを固定")
    a = p.parse_args()
    green, pl = generate(a.output, a.seed, a.green)
    print(f"world: {a.output}\ngreen label: {green}")
    for k, v in pl.items():
        print(f"  {k}: x={v[0]} y={v[1]} yaw={v[2]}")


if __name__ == "__main__":
    main()

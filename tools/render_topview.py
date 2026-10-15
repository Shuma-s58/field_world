#!/usr/bin/env python3
"""generate_world.py と同じ配置ロジックで、上から見たイメージ図(PNG)を描く。"""
import argparse
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

from field_world import generate_world as g


def box_patch(x, y, yaw, color, ax):
    t = Affine2D().rotate(yaw).translate(x, y) + ax.transData
    return Rectangle((-g.BOX_L / 2, -g.BOX_W / 2), g.BOX_L, g.BOX_W, facecolor=color,
                     edgecolor="black", linewidth=0.8, transform=t, zorder=3)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--green", choices=sorted(g.AREAS), default=None)
    p.add_argument("-o", "--output", default="field_topview.png")
    a = p.parse_args()

    # generate() と同じ乱数の使い方(緑ラベル→エリア順の配置)を再現するため実際に生成して読む
    tmp = Path(a.output).with_suffix(".world")
    green, pl = g.generate(str(tmp), a.seed, a.green)
    tmp.unlink()
    tmp.with_suffix(".answer.yaml").unlink()

    w, h = g.AREA_SIZE
    fig, ax = plt.subplots(figsize=(18, 4.4), dpi=150)
    for label, (cx, cy) in g.AREAS.items():
        ax.add_patch(Rectangle((cx - w / 2 - 0.05, cy - h / 2 - 0.05), w + 0.1, h + 0.1,
                               facecolor="#1a1a1a", zorder=1))
        ax.add_patch(Rectangle((cx - w / 2, cy - h / 2), w, h,
                               facecolor="#d9d9d9", zorder=2))
        ax.text(cx, cy + h / 2 + 0.35, f"Area {label}", ha="center", va="bottom",
                fontsize=12, fontweight="bold")
        x, y, yaw = pl[f"blue_box_{label}"]
        ax.add_patch(box_patch(x, y, yaw, "#1e46c8", ax))
        ax.text(x, y + 0.7, f"box {label}", ha="center",
                va="bottom", color="#1e46c8", fontsize=9, fontweight="bold", zorder=4)

    gx, gy = g.GREEN_POS
    ax.add_patch(box_patch(gx, gy, 0.0, "#1ea03c", ax))
    ax.text(gx, gy - 0.5, green, ha="center", va="top", color="#1ea03c", fontsize=11,
            fontweight="bold", zorder=4)
    ax.annotate(f"Green box\n(label changes every run: now '{green}')",
                xy=(gx, gy + 0.2), xytext=(gx, gy + h / 2 + 3.2), ha="center", fontsize=10,
                arrowprops=dict(arrowstyle="->"))

    ax.plot([0], [0], marker="", linestyle="")
    xmax = max(abs(cx) for cx, _ in g.AREAS.values()) + w / 2 + 1.0
    ax.set_xlim(-xmax, xmax)
    sx, sy, syaw = g.ROBOT_START
    ax.annotate("", xy=(sx + 1.6 * math.cos(syaw), sy + 1.6 * math.sin(syaw)), xytext=(sx, sy),
                arrowprops=dict(arrowstyle="-|>", color="tab:red", lw=2), zorder=5)
    ax.plot([sx], [sy], marker="o", color="tab:red", zorder=5)
    ax.annotate(f"robot start ({sx:g}, {sy:g}) facing green box", xy=(sx, sy),
                xytext=(sx + 1.0, sy + 0.3), fontsize=8, color="tab:red")
    ax.set_ylim(-3.2, g.AREA_Y + h / 2 + 3.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, linewidth=0.3, alpha=0.5, zorder=0)
    ax.set_title("field_world  top view  (blue boxes: random position in each area, label = area name)",
                 fontsize=11)
    fig.tight_layout()
    fig.savefig(a.output)
    print("saved", a.output, "green =", green)


if __name__ == "__main__":
    main()

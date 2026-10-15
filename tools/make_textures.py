#!/usr/bin/env python3
"""コンテナの色マテリアルと、白ラベル(宅配ボックス / 配達先 / 集荷ボックス)のテクスチャを生成する。

日本語フォントが必要(Noto Sans CJK)。生成物(PNG, .material)はパッケージに同梱済みなので、
作り直したいときだけ実行すればよい。
"""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent / "models/label_materials/materials"
TEX, SCR = ROOT / "textures", ROOT / "scripts"
TEX.mkdir(parents=True, exist_ok=True)
SCR.mkdir(parents=True, exist_ok=True)
for old in TEX.glob("*.png"):
    old.unlink()

FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
LATIN = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
BLUE = (20, 70, 180)


def font(size, path=FONT):
    return ImageFont.truetype(path, size)


def paper(w, h, header, lines, big=None):
    """白い紙。上に小さく「つくばチャレンジ」、中央に文字、(あれば)大きな英字。"""
    img = Image.new("RGB", (w, h), (250, 250, 250))
    d = ImageDraw.Draw(img)
    d.rectangle([1, 1, w - 2, h - 2], outline=(190, 190, 190), width=2)
    d.text((w * 0.06, h * 0.07), header, font=font(int(h * 0.075)), fill=BLUE)
    y = h * 0.22
    for text, ratio in lines:
        d.text((w / 2, y), text, font=font(int(h * ratio)), fill=(20, 20, 20), anchor="ma")
        y += h * ratio * 1.25
    if big:
        d.text((w / 2, h * 0.90), big, font=font(int(h * 0.50), LATIN), fill=(20, 20, 20),
               anchor="ms")   # ベースライン基準。b, d の縦棒が上の文字に当たらない大きさ
    return img


HEAD = "つくばチャレンジ"
mat = []

# --- コンテナ本体の色 ---------------------------------------------------------
for name, rgb in {"Blue": (0.10, 0.35, 0.80), "Green": (0.10, 0.65, 0.25),
                  "GreenLid": (0.14, 0.72, 0.30)}.items():
    r, g, b = rgb
    mat.append(f"""material Container/{name}
{{
  technique
  {{
    pass
    {{
      ambient {r} {g} {b} 1
      diffuse {r} {g} {b} 1
      specular 0.2 0.2 0.2 1 12
    }}
  }}
}}
""")


def textured(mname, fname):
    mat.append(f"""material {mname}
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1
      texture_unit
      {{
        texture {fname}
      }}
    }}
  }}
}}
""")


# --- ラベル(0.20 x 0.15 m の紙: 512x384) --------------------------------------
for ch in "abcd":
    paper(512, 384, HEAD, [("宅配ボックス", 0.11)], big=ch).save(TEX / f"delivery_{ch}.png")
    textured(f"Paper/Delivery_{ch}", f"delivery_{ch}.png")
    paper(512, 384, HEAD, [("配達先", 0.13)], big=ch).save(TEX / f"dest_{ch}.png")
    textured(f"Paper/Dest_{ch}", f"dest_{ch}.png")

# --- 集荷ボックスのラベル(0.14 x 0.20 m の紙・縦長: 358x512) ---------------------
paper(358, 512, HEAD, [("集荷", 0.16), ("ボックス", 0.12)]).save(TEX / "pickup.png")
textured("Paper/Pickup", "pickup.png")

(SCR / "label.material").write_text("\n".join(mat))
print("textures:", len(list(TEX.glob('*.png'))), " materials:", len(mat))

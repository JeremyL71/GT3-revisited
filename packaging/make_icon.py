#!/usr/bin/env python3
"""Draws the GT3 Revisited application icon (original artwork, no game assets).

Writes packaging/icon.png (1024x1024), packaging/icon.ico (Windows) and the
Android launcher icons. Needs Pillow:  python3 packaging/make_icon.py
"""
import math
import os

from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
S = 1024


def lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(len(a)))


def road_points(offset, n=200):
    # A sweeping right-hander from the bottom left to the top right.
    pts = []
    for i in range(n + 1):
        t = i / n
        x = 120 + 820 * t + 140 * math.sin(t * math.pi) * 0.35
        y = 930 - 820 * (t ** 1.35) - 180 * math.sin(t * math.pi)
        # Normal offset (perpendicular to the curve direction).
        t2 = min(1.0, t + 1e-3)
        x2 = 120 + 820 * t2 + 140 * math.sin(t2 * math.pi) * 0.35
        y2 = 930 - 820 * (t2 ** 1.35) - 180 * math.sin(t2 * math.pi)
        dx, dy = x2 - x, y2 - y
        ln = math.hypot(dx, dy) or 1
        pts.append((x - dy / ln * offset, y + dx / ln * offset))
    return pts


def draw(size=S):
    img = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    bg = Image.new("RGBA", (S, S))
    px = bg.load()
    top, bottom = (36, 40, 58), (12, 13, 19)
    for y in range(S):
        c = lerp(top, bottom, y / (S - 1))
        for x in range(S):
            px[x, y] = c + (255,)
    mask = Image.new("L", (S, S), 0)
    ImageDraw.Draw(mask).rounded_rectangle((32, 32, S - 32, S - 32), radius=200, fill=255)
    img.paste(bg, (0, 0), mask)

    road = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    d = ImageDraw.Draw(road)
    d.polygon(road_points(-125) + road_points(125)[::-1], fill=(58, 60, 68, 255))
    d.line(road_points(-112), fill=(235, 235, 240, 255), width=16, joint="curve")
    d.line(road_points(112), fill=(235, 235, 240, 255), width=16, joint="curve")
    # Red/white kerb on the inside of the corner.
    kerb = road_points(150, 60)
    for i in range(len(kerb) - 1):
        d.line([kerb[i], kerb[i + 1]], fill=(220, 50, 40, 255) if i % 2 else (240, 240, 240, 255), width=26)
    # Yellow dashed centre line.
    centre = road_points(0, 120)
    for i in range(0, len(centre) - 1, 6):
        d.line([centre[i], centre[min(i + 3, len(centre) - 1)]], fill=(255, 196, 40, 255), width=18)
    road_mask = Image.new("L", (S, S), 0)
    ImageDraw.Draw(road_mask).rounded_rectangle((32, 32, S - 32, S - 32), radius=200, fill=255)
    img.paste(road, (0, 0), Image.composite(road.getchannel("A"), Image.new("L", (S, S), 0), road_mask))

    # Rim highlight.
    ImageDraw.Draw(img).rounded_rectangle((34, 34, S - 34, S - 34), radius=198, outline=(255, 196, 40, 255), width=10)
    return img if size == S else img.resize((size, size), Image.LANCZOS)


def main():
    icon = draw()
    icon.save(os.path.join(HERE, "icon.png"))
    icon.save(os.path.join(HERE, "icon.ico"), sizes=[(16, 16), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
    res = os.path.join(ROOT, "android", "app", "src", "main", "res")
    for name, px in {"mdpi": 48, "hdpi": 72, "xhdpi": 96, "xxhdpi": 144, "xxxhdpi": 192}.items():
        out = os.path.join(res, "mipmap-" + name)
        os.makedirs(out, exist_ok=True)
        icon.resize((px, px), Image.LANCZOS).save(os.path.join(out, "ic_launcher.png"))
    print("icons written")


if __name__ == "__main__":
    main()

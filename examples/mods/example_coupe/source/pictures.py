# Dealer picture (640x448) and icon (51x27) from a car-viewer screenshot (README.md "Pictures").
#   python3 pictures.py shot.png ../images/concept_coupe.png ../images/concept_coupe_thumb.png
# Needs Pillow (pip install pillow).
import sys
from PIL import Image
src, pic, thumb = sys.argv[1:4]
im = Image.open(src).convert('RGB')
W, H = im.size
bg = im.getpixel((5, H // 2))
# Bounding box of everything that isn't the backdrop.
px = im.load(); xs = []; ys = []
for y in range(0, H, 2):
    for x in range(0, W, 2):
        p = px[x, y]
        if sum(abs(p[i] - bg[i]) for i in range(3)) > 24: xs.append(x); ys.append(y)
x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
def crop(aspect, margin, dx=0.0, dy=0.0):  # dx, dy: where the car sits, as a fraction of the crop
    w = (x1 - x0) * margin; h = w / aspect
    if h < (y1 - y0) * margin: h = (y1 - y0) * margin; w = h * aspect
    ox, oy = cx - dx * w, cy - dy * h
    return im.crop((int(ox - w / 2), int(oy - h / 2), int(ox + w / 2), int(oy + h / 2)))
# The car a little right and up: the name and price sit lower left.
crop(640 / 448, 1.35, 0.07, -0.07).resize((640, 448), Image.LANCZOS).save(pic, optimize=True)
crop(51 / 27, 1.04).resize((51, 27), Image.LANCZOS).save(thumb, optimize=True)
print('car box', x0, y0, x1, y1)

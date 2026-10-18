# Prints a body's size as the game will see it: run it on the finished .glb.
#   python3 measure_glb.py models/concept_coupe.glb
# glTF axes: x across the car, y up, z along it (nose at +z). 'half width w/o mirrors' x 2 = [body] width_mm.
import json, struct, sys
d = open(sys.argv[1], 'rb').read(); o = 12
while o + 8 <= len(d):
    ln, ty = struct.unpack_from('<II', d, o)
    if ty == 0x4E4F534A: doc = json.loads(d[o+8:o+8+ln])
    elif ty == 0x004E4942: b = d[o+8:o+8+ln]
    o += 8 + ((ln + 3) & ~3)
pts = []
for p in doc['meshes'][0]['primitives']:
    a = doc['accessors'][p['attributes']['POSITION']]; bv = doc['bufferViews'][a['bufferView']]
    off = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
    name = doc['materials'][p['material']]['name']
    for i in range(a['count']):
        pts.append((name,) + struct.unpack_from('<3f', b, off + 12 * i))
xs = [p[1] for p in pts]; ys = [p[2] for p in pts]; zs = [p[3] for p in pts]
print('x', min(xs), max(xs), 'y', min(ys), max(ys), 'z(front +)', min(zs), max(zs))
# width without the mirrors: max |x| of points below 0.85 m or away from the mirror zone
w = max(abs(p[1]) for p in pts if p[0] != 'mirror_chrome' and not (0.75 < p[2] < 1.0 and abs(p[1]) > 1.0))
print('half width w/o mirrors', w)
for lo, hi in ((0, 0.3), (0.3, 0.6), (0.6, 0.8), (0.8, 1.0)):
    sel = [abs(p[1]) for p in pts if lo <= p[2] < hi]
    print('band', lo, hi, 'half width', max(sel))
print('ground clearance (lowest point)', min(ys), 'length', max(zs) - min(zs))

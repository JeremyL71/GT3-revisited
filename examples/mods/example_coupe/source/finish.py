# Second pass (plain Python, no dependencies): sets the GT3 Revisited materials on the
# Blender export, embeds a home-made licence plate texture and repacks the GLB.
#   python3 -I finish.py <in.glb> <out.glb> [plate.png]
import json, struct, sys, zlib

src, dst = sys.argv[1], sys.argv[2]
plate_out = sys.argv[3] if len(sys.argv) > 3 else None

# --- the plate: "GT3R" on a pale plate, drawn from a 5x7 pixel font -----------------
GLYPHS = {
    'G': ["01110", "10001", "10000", "10111", "10001", "10001", "01111"],
    'T': ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    '3': ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    'R': ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
}
W, H = 256, 64
def plate_png(text="GT3R"):
    bg, ink, edge = (232, 232, 224), (24, 28, 44), (40, 40, 40)
    px = [[bg for _ in range(W)] for _ in range(H)]
    for y in range(H):
        for x in range(W):
            if x < 3 or y < 3 or x >= W - 3 or y >= H - 3: px[y][x] = edge
            elif x < 6 or y < 6 or x >= W - 6 or y >= H - 6: px[y][x] = bg
    sx, sy = 7, 6                       # scale of one font pixel (taller than wide, like plate type)
    gw, gap = 5 * sx, 2 * sx
    total = len(text) * gw + (len(text) - 1) * gap
    x0, y0 = (W - total) // 2, (H - 7 * sy) // 2
    for i, ch in enumerate(text):
        for r, row in enumerate(GLYPHS[ch]):
            for c, bit in enumerate(row):
                if bit == '1':
                    for yy in range(y0 + r * sy, y0 + (r + 1) * sy):
                        for xx in range(x0 + i * (gw + gap) + c * sx, x0 + i * (gw + gap) + (c + 1) * sx):
                            px[yy][xx] = ink
    raw = b''.join(b'\x00' + bytes(v for p in row for v in p) for row in px)
    def chunk(t, d): return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xFFFFFFFF)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', W, H, 8, 2, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))

# --- materials: base colours as the game should show them (it uses them as given) ----
# name: (rgba, metallic, roughness, doubleSided)
MATS = {
    'body_paint':           ((0.62, 0.07, 0.09, 1), 0.0, 0.3, True),   # default; [paints] override it
    'trim_gloss_black':     ((0.07, 0.07, 0.08, 1), 1.0, 0.15, True),
    'window_glass':         ((0.10, 0.12, 0.14, 0.62), 0.0, 0.0, True), # alpha < 1: blended
    'headlamp':             ((0.92, 0.95, 1.00, 1), 0.0, 0.2, False),
    'taillamp':             ((0.60, 0.04, 0.04, 1), 0.0, 0.2, False),
    'indicator_amber':      ((0.80, 0.42, 0.06, 1), 0.0, 0.3, False),
    'mirror_chrome':        ((0.78, 0.80, 0.83, 1), 1.0, 0.05, False),
    'plate':                ((1, 1, 1, 1), 0.0, 0.8, False),
    'chassis_dark':         ((0.06, 0.06, 0.06, 1), 0.0, 0.8, True),
    'trim_satin_metal':     ((0.50, 0.51, 0.53, 1), 1.0, 0.4, False),
    'trim_black':           ((0.05, 0.05, 0.05, 1), 0.0, 0.7, True),
    'interior_dark':        ((0.09, 0.09, 0.10, 1), 0.0, 0.8, True),
    'dashboard':            ((0.06, 0.06, 0.07, 1), 0.0, 0.7, False),
    'interior_leather_red': ((0.42, 0.07, 0.06, 1), 0.0, 0.6, True),
}

d = open(src, 'rb').read()
o = 12; doc = None; binc = b''
while o + 8 <= len(d):
    ln, ty = struct.unpack_from('<II', d, o)
    if ty == 0x4E4F534A: doc = json.loads(d[o + 8:o + 8 + ln])
    elif ty == 0x004E4942: binc = d[o + 8:o + 8 + ln]
    o += 8 + ((ln + 3) & ~3)

png = plate_png()
if plate_out: open(plate_out, 'wb').write(png)
views = [binc[v.get('byteOffset', 0):v.get('byteOffset', 0) + v['byteLength']] for v in doc['bufferViews']]

# Blender 5.2's exporter leaves the slot-0 material off its primitive here (the importer's
# variant bookkeeping); that primitive is the body paint (prepare.py puts it in slot 0).
bare = [p for m in doc['meshes'] for p in m['primitives'] if 'material' not in p]
assert len(bare) <= 1, 'more than one primitive without a material'
for p in bare:
    doc['materials'].append({'name': 'body_paint'})
    p['material'] = len(doc['materials']) - 1

# Keep only the materials the mesh uses (the importer carried the paint variants along).
used = sorted({p['material'] for m in doc['meshes'] for p in m['primitives']})
remap = {old: new for new, old in enumerate(used)}
doc['materials'] = [doc['materials'][i] for i in used]
for m in doc['meshes']:
    m.pop('extensions', None); m.pop('extras', None)
    for p in m['primitives']:
        p['material'] = remap[p['material']]
        p.pop('extensions', None); p.pop('extras', None)

for m in doc['materials']:
    name = m['name']
    if name not in MATS: raise SystemExit('unknown material ' + name)
    rgba, metal, rough, ds = MATS[name]
    tex = m.get('pbrMetallicRoughness', {}).get('baseColorTexture')
    new = {'name': name, 'pbrMetallicRoughness': {'baseColorFactor': list(rgba), 'metallicFactor': metal, 'roughnessFactor': rough}}
    if name == 'plate':
        if tex is None: raise SystemExit('plate lost its texture')
        new['pbrMetallicRoughness']['baseColorTexture'] = {'index': tex['index']}
    if rgba[3] < 1: new['alphaMode'] = 'BLEND'
    if ds: new['doubleSided'] = True
    m.clear(); m.update(new)

# One image (the plate), one texture, one sampler.
assert len(doc.get('images', [])) == 1, doc.get('images')
img = doc['images'][0]
views[img['bufferView']] = png
doc['images'] = [{'name': 'plate', 'mimeType': 'image/png', 'bufferView': img['bufferView']}]
doc['textures'] = [{'source': 0, 'sampler': 0}]
doc['samplers'] = [{'magFilter': 9729, 'minFilter': 9987, 'wrapS': 33071, 'wrapT': 33071}]
for m in doc['materials']:
    t = m['pbrMetallicRoughness'].get('baseColorTexture')
    if t: t['index'] = 0
doc.pop('extensionsUsed', None); doc.pop('extensionsRequired', None); doc.pop('extensions', None)
for n in doc['nodes']: n.pop('extras', None)
doc['asset'] = {'version': '2.0', 'generator': 'GT3 Revisited example_coupe (Blender + finish.py)',
                'copyright': '2024 Darmstadt Graphics Group GmbH (CC-BY 4.0); modified for GT3 Revisited'}

out = bytearray()
for i, v in enumerate(views):
    while len(out) % 4: out.append(0)
    doc['bufferViews'][i]['byteOffset'] = len(out)
    doc['bufferViews'][i]['byteLength'] = len(v)
    out += v
while len(out) % 4: out.append(0)
doc['buffers'] = [{'byteLength': len(out)}]
js = json.dumps(doc, separators=(',', ':')).encode()
while len(js) % 4: js += b' '
total = 12 + 8 + len(js) + 8 + len(out)
glb = b'glTF' + struct.pack('<II', 2, total) + struct.pack('<II', len(js), 0x4E4F534A) + js + struct.pack('<II', len(out), 0x004E4942) + bytes(out)
open(dst, 'wb').write(glb)
acc = doc['accessors']
tris = sum(acc[p['indices']]['count'] // 3 for m in doc['meshes'] for p in m['primitives'])
verts = sum(acc[p['attributes']['POSITION']]['count'] for m in doc['meshes'] for p in m['primitives'])
print('wrote', dst, len(glb), 'bytes,', tris, 'triangles,', verts, 'vertices,', len(doc['materials']), 'materials')

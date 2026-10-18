# Blender (background) pass: CarConcept.glb -> a game-ready body for GT3 Revisited.
#   blender -b --factory-startup --python prepare.py -- <in.glb> <out.glb>
# Deletes wheels/brakes/axles/engine/under-hood/small interior parts and the steering
# emblem, decimates, joins into one mesh, renames materials for the loader, moves the
# origin to the wheelbase midpoint on the ground. Materials are finalised by finish.py.
import bpy, bmesh, sys, math
from mathutils import Vector

argv = sys.argv[sys.argv.index('--') + 1:]
src, dst = argv[0], argv[1]
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=src)

objs = bpy.data.objects
def tris(o):
    return sum(len(p.vertices) - 2 for p in o.data.polygons) if o.type == 'MESH' else 0
before = sum(tris(o) for o in objs)

# Wheel centres (before deleting them): the origin goes to the wheelbase midpoint.
wc = {n: objs[n].matrix_world.translation.copy() for n in ('WheelFrontL', 'WheelFrontR', 'WheelRearL', 'WheelRearR')}
front_y = (wc['WheelFrontL'].y + wc['WheelFrontR'].y) / 2
rear_y = (wc['WheelRearL'].y + wc['WheelRearR'].y) / 2
mid_y = (front_y + rear_y) / 2
print('wheelbase', rear_y - front_y, 'front track', wc['WheelFrontL'].x - wc['WheelFrontR'].x,
      'rear track', wc['WheelRearL'].x - wc['WheelRearR'].x, 'wheel z', wc['WheelFrontL'].z)

# Parts the game doesn't need: the wheels, tyres and brakes (the engine draws its own),
# hidden mechanicals, small interior parts, the wiper blades (thin: unreadable once
# decimated) and the steering-wheel emblem (a trademark).
DELETE_PREFIX = ('BodyWindshieldWipers', 'Wheel', 'Axles', 'Engine', 'BodyHoodInterior', 'BodyHoodUnder', 'InteriorPedal',
                 'InteriorSteeringEmblem', 'InteriorRearHatch', 'InteriorFloormats', 'InteriorSteeringHandle',
                 'InteriorDoorR04', 'InteriorDoorR05', 'InteriorDoorR06', 'InteriorDoorL04', 'InteriorDoorL05',
                 'InteriorDoorL06', 'Mesh_')
doomed = set()
for o in list(objs):
    if o.name.startswith(DELETE_PREFIX) and o.name != 'BodyWindshieldWipersBase':
        doomed.add(o)
        doomed.update(o.children_recursive)
for o in doomed:
    bpy.data.objects.remove(o, do_unlink=True)
meshes = [o for o in objs if o.type == 'MESH']

# Bake every transform, then drop parents.
for o in meshes:
    mw = o.matrix_world.copy()
    o.parent = None
    o.matrix_world = mw
for o in [o for o in objs if o.type != 'MESH']:
    bpy.data.objects.remove(o, do_unlink=True)
bpy.ops.object.select_all(action='DESELECT')
for o in meshes:
    o.select_set(True)
bpy.context.view_layer.objects.active = meshes[0]
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# Decimation ratio per object (by name); small parts are left alone.
def ratio(o):
    n = o.name
    t = tris(o)
    if t < 400: return 1.0
    if n.startswith('Interior'): return 0.3
    if 'Window' in n or 'Windshield' in n or 'Rearwindow' in n: return 0.3
    if n.startswith(('BodyHood', 'BodyRearPanels', 'BodyPanelsColor2')): return 0.32
    return 0.45

for o in meshes:
    me = o.data
    # Custom normals from the import would be wrong after decimation: recomputed below.
    if me.has_custom_normals:
        bpy.context.view_layer.objects.active = o
        bpy.ops.mesh.customdata_custom_splitnormals_clear()
    # Only the licence plate keeps a UV set (its texture); everything else is flat colour.
    keep_uv = o.name.startswith('License')
    for uv in list(me.uv_layers):
        if not keep_uv or uv != me.uv_layers[0]:
            me.uv_layers.remove(uv)
    for ca in list(me.color_attributes):
        me.color_attributes.remove(ca)
    # Merge the duplicate vertices of UV seams so the decimator sees one surface.
    bm = bmesh.new()
    bm.from_mesh(me)
    if not keep_uv:
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bm.to_mesh(me)
    bm.free()
    r = ratio(o)
    if r < 1:
        m = o.modifiers.new('dec', 'DECIMATE')
        m.decimate_type = 'COLLAPSE'
        m.ratio = r
        m.use_collapse_triangulate = True
        bpy.context.view_layer.objects.active = o
        bpy.ops.object.modifier_apply(modifier=m.name)

# One object.
bpy.ops.object.select_all(action='DESELECT')
for o in meshes:
    o.select_set(True)
bpy.context.view_layer.objects.active = meshes[0]
bpy.ops.object.join()
body = bpy.context.view_layer.objects.active
body.name = 'body'
body.data.name = 'body'

# Origin: wheelbase midpoint on the ground (the tyres' contact patch is z = 0 already).
for v in body.data.vertices:
    v.co.y -= mid_y
# Flat/smooth shading by angle: hard creases stay sharp, panels stay smooth.
bpy.context.view_layer.objects.active = body
body.select_set(True)
bpy.ops.object.shade_smooth_by_angle(angle=math.radians(40), keep_sharp_edges=False)

# Material renames for the loader (finish.py sets their colours).
RENAME = {
    'Paint 1 Carmine': 'body_paint',          # 'paint'/'body': takes the chosen paint colour
    'Paint 2 Carmine': 'trim_gloss_black',    # roof, sills, mirror caps, bonnet vent: two-tone
    'Glass': 'window_glass',
    'Headlight': 'headlamp',
    'Brakelight': 'taillamp',
    'Signallight': 'indicator_amber',
    'Mirror': 'mirror_chrome',
    'License': 'plate',
    'Mechanical': 'chassis_dark',
    'Hardware': 'trim_satin_metal',
    'Material_2': 'trim_black',
    'Interior 1': 'interior_dark',
    'Interior 2': 'interior_dark',
    'Panel Sides': 'interior_dark',
    'Floormat': 'interior_dark',
    'Dashboard': 'dashboard',
    'Interior 3 Carmine': 'interior_leather_red',
}
mats = body.data.materials
# Object-linked slots (the importer's variant setup) become plain mesh slots.
for i, slot in enumerate(body.material_slots):
    eff = slot.material
    if slot.link == 'OBJECT':
        slot.link = 'DATA'
        mats[i] = eff
targets = {}
for i, m in enumerate(mats):
    if m is None: continue
    new = RENAME.get(m.name)
    if new is None:
        print('UNMAPPED MATERIAL', m.name)
        continue
    if new not in targets:
        nm = bpy.data.materials.new(new)
        targets[new] = nm
    mats[i] = targets[new]
# The plate keeps its UVs; give it an image node so the exporter writes TEXCOORD_0.
plate = targets.get('plate')
img = bpy.data.images.new('plate', 8, 2)
tex = plate.node_tree.nodes.new('ShaderNodeTexImage')
tex.image = img
bsdf = plate.node_tree.nodes['Principled BSDF']
plate.node_tree.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])

# Merge slots that now share a material, then drop the unused ones.
first = {}
for i, m in enumerate(mats):
    first.setdefault(m.name if m else None, i)
for poly in body.data.polygons:
    m = mats[poly.material_index]
    poly.material_index = first[m.name if m else None]
bpy.ops.object.material_slot_remove_unused()
body.data.validate(verbose=False)
# Drop the paint variants (KHR_materials_variants) and every material the body no longer uses.
for holder, prop in ((bpy.context.scene, 'gltf2_KHR_materials_variants_variants'), (body.data, 'gltf2_variant_mesh_data')):
    if hasattr(holder, prop): getattr(holder, prop).clear()
keep = {m.name for m in body.data.materials}
for m in list(bpy.data.materials):
    if m.name not in keep: bpy.data.materials.remove(m)
for im in list(bpy.data.images):
    if im.name != 'plate': bpy.data.images.remove(im)

after = sum(len(p.vertices) - 2 for p in body.data.polygons)
lo = Vector((1e9,) * 3); hi = Vector((-1e9,) * 3)
for v in body.data.vertices:
    for k in range(3): lo[k] = min(lo[k], v.co[k]); hi[k] = max(hi[k], v.co[k])
print('TRIS before', before, 'after', after)
print('BBOX', tuple(round(x, 4) for x in lo), tuple(round(x, 4) for x in hi))
print('MATS', [m.name for m in body.data.materials])

bpy.ops.export_scene.gltf(filepath=dst, export_format='GLB', use_selection=False, export_yup=True,
                          export_texcoords=True, export_normals=True, export_materials='EXPORT',
                          export_vertex_color='NONE', export_image_format='AUTO', export_extras=False,
                          export_apply=True)

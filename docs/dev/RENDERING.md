# Rendering

The 3D renderer lives in `src/render/` and is built on **SDL_GPU**: Metal on
macOS/iOS, Vulkan on Linux, Android and Windows. The 2D UI (`Ui`,
`SDL_Renderer`) keeps working unchanged on top of it.

## Frame structure

```
App::init
  GpuContext::create(window)          SDL_CreateGPUDevice(SPIR-V | MSL)
    SDL_CreateGPURenderer(device, window)   2D renderer on the same device;
                                            it owns the swapchain
  (failure) -> SDL_CreateRenderer(window, nullptr), 2D only

each frame (App::iterate)
  Ui::clear
  Screen::render
    View3D::draw(rect, camera, models)    own command buffer:
      render pass -> offscreen RGBA8 (+MSAA resolve, +depth)
      submit
      SDL_RenderTexture(wrapper)          wrapper = SDL_Texture around the GPU texture
    ui.text(...)                          UI on top
  SDL_RenderPresent                       2D command buffer, submitted after ours
```

The 3D views render into offscreen textures. Each view's command buffer is
submitted before the 2D renderer's, so GPU ordering is guaranteed. The 2D
renderer then composites the view as an ordinary texture
(`SDL_PROP_TEXTURE_CREATE_GPU_TEXTURE_POINTER`). This means:

- Several 3D views per screen work (e.g. a car preview inside a menu).
- `--shot` and F12 (`SDL_RenderReadPixels`) include the 3D layer without
  extra code.
- HiDPI: the offscreen target is sized in physical pixels (logical
  rectangle × logical-presentation scale), so 3D stays sharp at 640×360 UI
  scale.
- Resizing is handled lazily: the target is recreated when the pixel size
  or MSAA level changes.

**SDL version:** this needs SDL ≥ 3.4 (`SDL_CreateGPURenderer`, wrapping GPU
textures). The code compiles against 3.2 (guarded by
`SDL_VERSION_ATLEAST(3, 4, 0)`) and then runs 2D-only. The build, CI and
Android all fetch `release-3.4.12` (`cmake/Dependencies.cmake`).

## Fallback

`GpuContext::create` returns nullptr, and the app logs it and continues
2D-only, when:

- SDL is older than 3.4;
- no GPU backend accepts SPIR-V or MSL (e.g. a Windows PC without Vulkan
  drivers: we don't ship DXIL yet);
- device, renderer, shader or sampler creation fails;
- the user disabled it: `[video] gpu=0` in `config.ini`, or `--no-gpu`.

`App::gpu()` is nullptr in that case. Screens must check it.
`ModelViewer` shows the list and statistics with a "3D renderer
unavailable" note.

## Configuration

| `config.ini [video]` | Command line | Default | Effect |
|---|---|---|---|
| `gpu` | `--no-gpu` | 1 | Use the SDL_GPU path |
| `msaa` | `--msaa N` | 4 | 1/2/4/8, clamped to device support |
| `vsync` | | 1 | Present with vsync |
| `gpu_debug` | | 0 | Backend validation layers |
| `render_scale` | `--render-scale X` | auto | 0.5..1.0: 3D views render at this fraction of their on-screen pixel size and are upscaled (linear) when composited; UI and HUD stay native. `auto` = 1.0 on desktop; on Android/iOS about 900 lines of the 16:9 game area (e.g. 0.63 on a 2560×1600 tablet, 0.70 on a 2856×1280 phone). See `src/platform/render_scale.*` |

## Shaders

Shaders are authored once in Vulkan GLSL (`src/render/shaders/model.vert`,
`model.frag`) and compiled **offline**. The outputs are committed, so CI and
the Android Gradle build need no shader tools:

```
GLSL --glslangValidator--> SPIR-V   (Vulkan)
SPIR-V --spirv-cross--msl-> MSL     (Metal, compiled from source at runtime)
```

To regenerate after editing a shader:

```bash
brew install glslang spirv-cross     # or the LunarG Vulkan SDK on Linux/Windows
python3 src/render/shaders/compile_shaders.py
```

This writes `src/render/shaders/generated/model_{vert,frag}.h`: a 4-byte
aligned SPIR-V byte array and the MSL source (entry point `main0`).

Binding conventions follow SDL_GPU's rules:

- Vertex uniforms: `set = 1`.
- Fragment textures/samplers: `set = 2`.
- Fragment uniforms: `set = 3`.
- MSL uses `--msl-decoration-binding`, so `[[buffer(0)]]`, `[[texture(0)]]`
  etc. match SDL's Metal slots. Vertex buffers start at 14 and are handled
  by SDL.

**D3D12 (DXIL)** is not generated yet: `dxc` is not readily available on
macOS. On Windows, SDL picks Vulkan because we only offer SPIR-V and MSL. To
add D3D12 later:

1. Run `spirv-cross --hlsl --shader-model 60` on the SPIR-V. Remap the
   bindings to SDL's D3D12 spaces: vertex `b0, space1`; fragment
   `t0/s0, space2` and `b0, space3`.
2. Compile with `dxc -T vs_6_0/ps_6_0` and add `SDL_GPU_SHADERFORMAT_DXIL`
   to the device and `make_shader`.

## Materials

There is one shader pair. Per-primitive state comes from
`scene::PrimitiveFlags`:

| Flag | Pipeline / shader effect |
|---|---|
| none | opaque, back-face culled (CCW = front), depth write |
| `kAlphaBlend` | src·α + dst·(1−α), drawn after opaque, in model order |
| `kAdditive` | src·α + dst, drawn after opaque |
| `kAlphaTest` | `discard` if α ≤ `Primitive::alpha_ref` when ≥ 0 (the GS AREF from the node program), else `DrawOptions::alpha_ref` (0.5) / `blend_alpha_ref` |
| `kAlphaBlend \| kAlphaTest \| kAlphaFailBlend` | GS AFAIL = RGB_ONLY (course foliage): drawn twice, the fragments passing the test in the opaque phase (with depth write), the failing ones blended in the blended phase without depth write |
| `kDoubleSided` | no culling; normals flipped for back faces |
| `kUnlit` | colour = texture × vertex colour, no lighting |
| `kEnvironmentMap` | placeholder reflection: sky/ground gradient mixed by `env_strength` |
| `kNoDepthWrite` | depth test on (LESS_OR_EQUAL, so second passes over the same vertices pass), depth write off |
| `kAdditive \| kEnvironmentMap` (textured, no `kUnlit`) | course road sheen: additive grey specular highlight of the texture, weighted by vertex alpha (`DrawOptions::sheen_strength`) |
| `kAdditive \| kEnvironmentMap \| kUnlit` (or untextured) | car reflection pass (PS2 ALPHA Cs·Ad + Cd): `env_color(reflect(view, n))` × min(vertex RGB × weight, 255/128) × `DrawOptions::reflection_strength`, added with weight = vertex alpha × texture alpha. The bound texture is the base pass's texture with its UVs; its alpha is the per-pixel reflectivity (0 in cut-outs). The per-vertex weight is the primitive's reflection rows `env_c0 + env_c1 × min(V·N, 0)` (decoded from the node program), else `1 + reflection_fresnel × min(V·N, 0)`. See docs/formats/car_model.md. |
| `DrawOptions::env_sphere` | car reflection passes sample this 2D sphere map in eye space (GT3's kernel: s = 0.5 + 0.5 Rx / m, t = 0.5 − 0.5 Ry / m, m = \|R + (0, 0, 1)\|) instead of the cube / gradient: the menu showrooms' fixed reflection image |
| `DrawOptions::quantize_rgb5` | every fragment stored at 5 bits per channel (c8 >> 3 << 3): set while the race renders GT3's reflection map (a PSMCT16 frame buffer) into the cube, so the reflection lookup filters between quantised texels as the GS does |
| `kMultiplyDest` | GS ALPHA 0x89 (Cd·As): destination × fragment alpha (prize cup rims) |
| textured `kEnvironmentMap` with `DrawItem::sphere_map` | prize scenes: the model's own sphere map, UVs from the reflected view vector in view space (GT3's reflection kernel), vertex colour × the reflection rows |
| `DrawItem::frame_alpha` (race course scenery) | the PS2's frame-buffer alpha: every pass writes its fragment alpha unblended (`AlphaWrite::Write`; `kKeepAlpha` = FBMSK 0xFF000000 keeps it), `kAlphaOnly` passes (FBMSK 0x00FFFFFF shadow masks) write only the alpha, and the road sheen is added as Cs·Ad (`BlendMode::DestAlphaAdd`, GS ALPHA 0x58). `kAlphaOnly` primitives of other items are not drawn |

Alpha is otherwise never written to the colour target (write mask RGB): the
3D view stays opaque when composited and in screenshots. When a
`frame_alpha` item wrote it, the view is composited with a colour-only blend
(the 2D frame keeps its own alpha).

Lit materials use one directional light plus ambient (`DrawOptions`), and up
to three more (`fill_*`, or `ItemLights` per item). Vertex colour is used
as-is (RGBA8 → 0..1). Decoders should map PS2's 0x80 = 1.0 convention
themselves.

### GT3 lighting (primitives with a valid `scene::Material`)

The VU formula (docs/formats/car_model.md): Me + La·Ma + Σ Ld·Md·max(N·L, 0) +
Σ Ls·Ms·max(N·H, 0)^p, times the packet scale (1 or 255/128), clamped to
255/128. Textured: MODULATE (texel × lit, 0x80 = 1.0). Untextured: the GS
shows the lit byte directly (255 = 1.0), so the shader multiplies by 128/255.
As on the VU, the formula is evaluated per vertex in `model.vert` (lights and
material in vertex uniform slot 1, `VertexLighting`), with GT3's infinite
viewer (H from the view's z axis, not the direction to each vertex), clamped per
vertex and interpolated across the triangle (Gouraud): highlights are as soft as
the original's on its coarse meshes.
`DrawItem::light_scale` (road shade) scales every light, fills included.
Light sources per scene:

| Scene | Lights |
|---|---|
| race (day) | Env0 car lights 0..3 enabled by Env0 +0x10 bits 0..3 (`race::SunParams::day_car_lights`): light 0 = sun, the others fills (often a negative-diffuse light from below); ambient = global ambient + the enabled lights' ambients; world space |
| race (point-light courses) | per car `ItemLights` (up to 4 nearby lamps as directional lights) |
| showroom | `menu/showroom_<kind>.fbi`: global ambient + 3 lights in eye space |
| prize scenes | the `.prm` lights in scene space (turned by the view), global ambient 0 |

Textures are uploaded as RGBA8 with a full GPU-generated mip chain and
linear/trilinear filtering with 4× anisotropy. There are four samplers for
the `clamp_u`/`clamp_v` combinations (otherwise repeat). `texture = -1`
binds a 1×1 white texture.

Out-of-range indices are dropped on upload. A primitive's `index_count` is
rounded down to whole triangles.

Wireframe draws a per-mesh line list of unique edges, built on upload. It
uses a slightly depth-biased projection, so it works without
`fillModeNonSolid` (often missing on Android Vulkan drivers).

## Large scenes

`DrawOptions` options used by the race scene (all off by default):

| Option | Effect |
|---|---|
| `frustum_cull` | skip meshes whose bounding sphere (`GpuModel::MeshInfo::center/radius`, computed on upload) is outside the frustum |
| `draw_distance` | skip meshes further than this; `DrawItem::draw_distance` overrides per item (0 = never) |
| `sort_blended` | blended primitives drawn back to front by mesh distance instead of model order |
| `fog`, `fog_color/start/end/max` | linear distance fog (not on background or helper items) |
| `blend_alpha_ref` | alpha-test threshold for blended `kAlphaTest` primitives |

`DrawItem::background` draws an item first, without depth writes, culling or
fog (sky domes; pass `translate(eye)` as the transform). `View3D::stats()`
returns meshes drawn/culled, draw calls and triangles of the last draw.
`math3d.h` has `rotate_x`, `rotate_z` and `rotate_quat`.

## API summary

| Header | What |
|---|---|
| `render/gpu_context.h` | `GpuContext` (device, 2D renderer, pipelines, samplers), `Uploader` |
| `render/gpu_model.h` | `GpuModel` (upload of a `scene::Model`), `model_stats()` |
| `render/view3d.h` | `View3D::draw(renderer, rect, camera, items, options)`, `DrawOptions`, `WireMode` |
| `render/camera.h` | `OrbitCamera` (pad, mouse, touch, turntable) |
| `render/math3d.h` | `Vec3`, `Mat4` (column-major), `perspective`, `look_at`, `inverse` |
| `render/test_models.h` | procedural test scenes, ground grid |
| `app/screen_model_viewer.h` | `make_model_viewer(title, items, loader)` |

Coordinate conventions:

- Right-handed, +Y up.
- Clip depth 0..1.
- `scene::Mesh::transform` is column-major and applied as
  `item.transform * mesh.transform`.
- `Model::compute_bounds` ignores mesh transforms. Decoders with
  transformed meshes should fill `bounds_*` themselves.

## Model viewer

`make_model_viewer(title, items, load)` shows a list and a 3D view. The
highlighted item loads after 0.25 s (or on Cross). If `load` throws, the
error message is shown.

Controls:

- List: Up/Down select, Cross enlarges the view, Square cycles wireframe
  (solid / solid+wire / wire), L1/R1 zoom, Triangle goes back.
- Enlarged view: stick/d-pad orbit, L1/R1 (or L2/R2, right stick) zoom,
  Cross toggles the turntable, Select cycles LOD (0..n, all), Start cycles
  cull override (material/none/back/front), L3 toggles the grid,
  Triangle/Circle return to the list.
- Mouse: left-drag orbits, right/middle-drag pans, wheel zooms.
- Touch: drag orbits, pinch zooms, tap a list item to load it (tap again to
  enlarge).

For screenshots, the `GT3R_VIEWER` environment variable presets state:

```bash
GT3R_VIEWER="item=1,full=1,wire=1,lod=-1,cull=0,spin=0,yaw=0.5,pitch=0.3,zoom=0.8,grid=1" \
  gt3r --screen viewer --shot out.png
```

The demo screen `viewer` ("3D viewer (test scene)") needs no game data, so
`--screen viewer` also works before a disc is chosen.

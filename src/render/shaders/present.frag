#version 450
// VR present pass (src/xr): a canvas into a headset swapchain image. The image is sRGB, so the
// hardware encodes what we write: writing the canvas's colour decoded stores its bytes as they
// are (the game's colours are display-referred, like the PS2's), and the compositor shows them
// as drawn. SDL_GPU binding rules: sampled textures in set 2, uniform buffers in set 3.

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 o_color;

layout(set = 2, binding = 0) uniform sampler2D u_canvas;

layout(set = 3, binding = 0) uniform PresentUniforms {
    vec4 mode; // x = 1: premultiplied alpha (the HUD panel): decode the unpremultiplied colour;
               // y = 1: a non-sRGB image (runtimes without sRGB swapchains): copy as is
};

vec3 decode(vec3 c) {
    return mix(c / 12.92, pow((c + 0.055) / 1.055, vec3(2.4)), step(vec3(0.04045), c));
}

void main() {
    vec4 c = texture(u_canvas, v_uv);
    if (mode.y > 0.5)
        o_color = mode.x > 0.5 ? c : vec4(c.rgb, 1.0);
    else if (mode.x > 0.5)
        o_color = vec4(c.a > 0.0 ? decode(c.rgb / c.a) * c.a : vec3(0.0), c.a);
    else
        o_color = vec4(decode(c.rgb), 1.0);
}

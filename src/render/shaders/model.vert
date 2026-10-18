#version 450
// Model vertex shader. SDL_GPU binding rules for SPIR-V vertex shaders:
// uniform buffers live in descriptor set 1.

layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;
layout(location = 3) in vec4 a_color;

layout(set = 1, binding = 0) uniform VertexUniforms {
    mat4 mvp;
    mat4 model;      // object -> world
    mat4 normal_mat; // inverse-transpose of model (upper 3x3 used)
} u;

// GT3's VU lighting, evaluated per vertex (set when the primitive has a scene::Material):
// the same rows as the fragment uniforms' GT3 lighting.
layout(set = 1, binding = 1) uniform VertexLighting {
    vec4 eye_dir;        // xyz: towards the viewer (world; the view's z axis: GT3's infinite
                         // viewer), w = 1: light this primitive per vertex
    vec4 light_dir[4];   // xyz: towards the light (world), w = 1 enabled
    vec4 light_diffuse[4];
    vec4 light_specular[4];
    vec4 light_ambient;  // rgb
    vec4 mat_ambient;
    vec4 mat_diffuse;    // w: scale (1, or 255/128)
    vec4 mat_specular;   // w: exponent (0 = no specular term)
    vec4 mat_emission;   // w: colour factor after the clamp (1, or 128/255 untextured)
} l;

layout(location = 0) out vec3 v_normal;
layout(location = 1) out vec2 v_uv;
layout(location = 2) out vec4 v_color;
layout(location = 3) out vec3 v_world;
layout(location = 4) out vec3 v_lit; // GT3 lighting factor (texel or 1 x this), per vertex

void main() {
    gl_Position = u.mvp * vec4(a_pos, 1.0);
    v_world = (u.model * vec4(a_pos, 1.0)).xyz;
    v_normal = mat3(u.normal_mat) * a_normal;
    v_uv = a_uv;
    v_color = a_color;
    // GT3: Me + La Ma + sum Ld Md max(N.L, 0) + sum Ls Ms max(N.H, 0)^p (H with the infinite
    // viewer), times the packet scale, clamped to 255/128 per vertex; the GS interpolates
    // the result across the triangle (Gouraud) and modulates the texel with it.
    v_lit = vec3(1.0);
    if (l.eye_dir.w > 0.5) {
        vec3 n = normalize(v_normal);
        vec3 E = normalize(l.eye_dir.xyz);
        vec3 lit = l.mat_emission.rgb + l.light_ambient.rgb * l.mat_ambient.rgb;
        for (int i = 0; i < 4; ++i) {
            if (l.light_dir[i].w < 0.5) continue;
            vec3 L = normalize(l.light_dir[i].xyz);
            lit += l.light_diffuse[i].rgb * l.mat_diffuse.rgb * max(dot(n, L), 0.0);
            if (l.mat_specular.w > 0.0)
                lit += l.light_specular[i].rgb * l.mat_specular.rgb *
                       pow(max(dot(n, normalize(L + E)), 0.0), l.mat_specular.w);
        }
        v_lit = min(lit * l.mat_diffuse.w, vec3(255.0 / 128.0)) * l.mat_emission.w;
    }
}

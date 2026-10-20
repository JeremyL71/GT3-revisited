#version 450
// VR present pass (src/xr): a triangle over the whole target from the vertex index alone,
// texture coordinates 0..1 from the top left (SDL_GPU's texture layout).

layout(location = 0) out vec2 v_uv;

void main() {
    vec2 p = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    v_uv = p;
    gl_Position = vec4(p.x * 2.0 - 1.0, 1.0 - p.y * 2.0, 0.0, 1.0);
}

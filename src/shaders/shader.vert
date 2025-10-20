#version 450
#extension GL_ARB_shader_draw_parameters : require

layout(location = 0) in vec4 inPosition;
layout(location = 1) in vec4 inColor;
layout(location = 2) in vec4 inNormal;

layout(set = 0, binding = 0, row_major, std140) uniform UBO {
    mat4 u_mvp;
} ubo;

struct PerDrawGPU {
    uint  clusterId;
    uint  lod;
    uvec2 _pad;
    vec4  color;
 };

layout(set = 0, binding = 2, std430) readonly buffer PerDrawBuf {
    PerDrawGPU  draws[];
};

layout(location = 0) out vec4 fragColor;
layout(location = 1) flat out uint drawMode;

void main() {

    uint drawID = uint(gl_BaseInstanceARB);  
    fragColor   = draws[drawID].color;
    drawMode = draws[drawID]._pad.x;
    gl_Position = inPosition * ubo.u_mvp;
}
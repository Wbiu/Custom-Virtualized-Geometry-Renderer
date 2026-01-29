#version 460 core

// Same binding index as your descriptor layout (3)
layout(binding = 3, rgba8) uniform readonly image2D rtImage;

layout(location = 0) out vec4 outColor;

void main()
{
    ivec2 coord = ivec2(gl_FragCoord.xy);
    vec4 c = imageLoad(rtImage, coord);
    outColor = c;
}

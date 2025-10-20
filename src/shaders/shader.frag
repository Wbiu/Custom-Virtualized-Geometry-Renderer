#version 450
layout(location = 0) out vec4 outColor;

layout(location = 0) in vec4 fragColor;
layout(location = 1) flat in uint drawMode;

vec3 randomColor(uint id) {
    float r = float(((id * 97) % 231) + 24) / 255.0; // Red component
    float g = float(((id * 57) % 231) + 24)/ 255.0; // Green component
    float b = float(((id * 33) % 231) + 24) / 255.0; // Blue component
    return vec3(r, g, b);
}

void main() {

    if(drawMode == 1)
    {
        outColor = vec4(randomColor(gl_PrimitiveID), 1.0);
    }
    else
    {
       outColor = fragColor;
    }

}
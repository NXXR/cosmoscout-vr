#version 330

in vec2 vTexCoords;
in vec4 vPosition;

uniform sampler2D iTexture;

layout(location = 0) out vec4 vOutColor;

void main() {
    vOutColor = texture(iTexture, vTexCoords);
    if (vOutColor.a == 0.0) discard;

    vOutColor.rgb /= vOutColor.a;
}  
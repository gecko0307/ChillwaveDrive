#version 400 core

layout (location = 0) in vec3 va_Vertex;
layout (location = 1) in vec3 va_Normal;

uniform mat4 modelViewMatrix;
uniform mat4 normalMatrix;
uniform mat4 projectionMatrix;
uniform mat4 invViewMatrix;
uniform mat4 prevModelViewMatrix;

out vec4 currPosition;
out vec4 prevPosition;

out vec3 eyePosition;
out vec3 worldPosition;
out vec3 worldNormal;

void main()
{
    vec4 pos = modelViewMatrix * vec4(va_Vertex, 1.0);
    eyePosition = pos.xyz;
    worldPosition = (invViewMatrix * pos).xyz;
    worldNormal = normalize(va_Vertex);
    
    currPosition = projectionMatrix * pos;
    prevPosition = projectionMatrix * prevModelViewMatrix * vec4(va_Vertex, 1.0);

    gl_Position = currPosition;
}

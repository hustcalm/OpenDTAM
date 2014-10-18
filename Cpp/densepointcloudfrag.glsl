#version 330
layout (location = 0) out vec4 outputColor;

in vec3 vColor;

void main()
{
  
  outputColor.xyz = vColor;
  
}

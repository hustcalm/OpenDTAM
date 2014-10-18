#version 330

layout (location=0) in vec3 position;
layout (location=1) in vec3 color;
layout (location=2) in vec3 normal;

uniform mat4 mvpMatrix;

out vec4 vPosition;
out vec3 vColor;

void main()
{
   gl_Position = mvpMatrix * vec4( position , 1 ); 
   
   vColor = color;
}
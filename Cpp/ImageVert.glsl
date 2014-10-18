#version 330

layout (location=0) in vec3 position;
layout (location=1) in vec2 texCoords;

uniform mat4 orthoMatrix;

out vec2 tcoords;

void main()
{
   vec4 vPosition = vec4( position , 1 ); 
   
   gl_Position = orthoMatrix * vPosition;
   
   tcoords = texCoords;
}
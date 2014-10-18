#version 330

layout (location = 0) out vec4 color;

in vec2 tcoords;

uniform sampler2D ReferenceTexture;

void main()
{
  color = texture( ReferenceTexture , tcoords );

//   color.xyz = vec3( 1.0 , 0 , 0 );
}
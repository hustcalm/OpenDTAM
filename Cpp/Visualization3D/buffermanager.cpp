#include "buffermanager.h"
#include "Visualization3D/openglhelper.h"

namespace vc{

	GLuint BufferManager::compileShaders( GLenum shaderType , const char *shaderSource )
	{
		GLuint shader = glCreateShader( shaderType );

	   GL_CHECK( glShaderSource( shader , 1 , &shaderSource , NULL) );
       GL_CHECK( glCompileShader( shader ) );

    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);


    if( status == GL_FALSE ) {
        GLchar emsg[1024];
        glGetShaderInfoLog(shader, sizeof(emsg), 0, emsg);
        fprintf(stderr, "Error compiling GLSL shader (%s): %s\n" , emsg );
        //fprintf(stderr, "Section: %s\n", sdefine);
        //fprintf(stderr, "Defines: %s\n", define);
        //fprintf(stderr, "Source: %s\n", sources[2]);

	qDebug()<<" error in compiling shaders "<< endl;

        exit(0);
    }

    return shader;
	
	}

}
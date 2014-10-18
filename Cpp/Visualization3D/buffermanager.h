#ifndef __BUFFERMANAGER_H__
#define __BUFFERMANAGER_H__

#include "external_includes/openglincludes.h"

#ifdef VC_QOPENGL_FUNCTIONS
#include "QOpenGLFunctions_3_3_Core"
#endif


namespace vc{


#ifdef VC_QOPENGL_FUNCTIONS
class BufferManager : public QOpenGLFunctions_3_3_Core
#else
class BufferManager 
#endif
{

public:


virtual void init() = 0;
virtual void render() = 0;

// virtual int getNumVertices() = 0;
// virtual int getNumFaces() = 0;

// protected:

 GLuint compileShaders( GLenum shaderType , const char *shaderSource );

};

}

#endif
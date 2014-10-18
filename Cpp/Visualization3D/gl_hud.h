//
//   Copyright 2013 Pixar
//
//   Licensed under the Apache License, Version 2.0 (the "Apache License")
//   with the following modification; you may not use this file except in
//   compliance with the Apache License and the following modification to it:
//   Section 6. Trademarks. is deleted and replaced with:
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor
//      and its affiliates, except as required to comply with Section 4(c) of
//      the License and to reproduce the content of the NOTICE file.
//
//   You may obtain a copy of the Apache License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the Apache License with the above modification is
//   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
//   KIND, either express or implied. See the Apache License for the specific
//   language governing permissions and limitations under the Apache License.
//

#ifndef GL_HUD_H
#define GL_HUD_H

#include "hud.h"

#include "external_includes/openglincludes.h"

#ifdef VC_QOPENGL_FUNCTIONS
#include "qopenglfunctions_3_3_core.h"
#endif

//#include <osd/opengl.h>

#ifdef VC_QOPENGL_FUNCTIONS
class GLhud : public Hud , public QOpenGLFunctions_3_3_Core
#else
class GLhud : public Hud
#endif
{
public:
    GLhud();
    ~GLhud();

    virtual void Init(int width, int height);

    virtual void Rebuild(int width, int height);

    virtual bool Flush();
    
private:
    GLuint _fontTexture;
    GLuint _vbo, _staticVbo;
    GLuint _vao, _staticVao;
    int _staticVboSize;

    GLint _program;
    GLint _mvpMatrix;
    GLint _aPosition, _aColor, _aUV;

protected:

	GLuint compileShader(GLenum shaderType, const char *source);
};

#endif // GL_HUD_H

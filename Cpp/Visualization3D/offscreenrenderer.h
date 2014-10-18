#ifndef __OFSCREENRENDERER_H__
#define __OFSCREENRENDERER_H__

#include "buffermanager.h"
#include "QOpenGLContext"
#include "QOffscreenSurface"


namespace vc{

class OffScreenRenderer
{


protected:

	QOpenGLContext *mContext;
	QOffscreenSurface *mFakeSurface;
	BufferManager *mBufferManager;

	int mWidth , mHeight;

 public:
 
	 OffScreenRenderer();

	 void setRenderBuffer( BufferManager *bufferManager );
	 void setDimension( int width , int height );

     void init();
     void render();

};

}

#endif
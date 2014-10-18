#include "offscreenrenderer.h"
#include "QSurfaceFormat"


namespace vc{

	OffScreenRenderer::OffScreenRenderer()
	{
	        QSurfaceFormat format;
		
		format.setProfile( QSurfaceFormat::CoreProfile );
		format.setMajorVersion( 4 );
		format.setMinorVersion( 1 );
	  
		mContext = new QOpenGLContext();
		mContext->setFormat( format );
		mContext->create();

		mFakeSurface = new QOffscreenSurface();
                //mFakeSurface->setFormat(context->format());
		mFakeSurface->setFormat( format );
                mFakeSurface->create();

		mContext->makeCurrent( mFakeSurface );  

		mBufferManager = 0;

		//default window size
		mWidth = 640;
		mHeight = 480;
	}

	void OffScreenRenderer::setRenderBuffer( BufferManager *bufferManager )
	{
		mBufferManager = bufferManager;
	}

	void OffScreenRenderer::setDimension( int width , int height )
	{
		mWidth = width;
		mHeight = height;
	}

	void OffScreenRenderer::init()
	{
		if( mBufferManager )
		{
		   if (!mContext->isValid())
              mContext->create();
 
            mContext->makeCurrent( mFakeSurface );

			mBufferManager->init();
		}
	}

	void OffScreenRenderer::render()
    {
        if (!mContext->isValid())
            mContext->create();
 
        mContext->makeCurrent( mFakeSurface );
 
		glViewport( 0 , 0 , mWidth , mHeight );
 
	
		if( mBufferManager )
		  mBufferManager->render();

        // We need to flush the contents to the FBO before posting
        // the texture to the other thread, otherwise, we might
        // get unexpected results.
 
		glFlush();
 
    }


}
#include "drawimage.h"
#include "Visualization3D/openglhelper.h"

namespace vc{

DrawImage::DrawImage()
{
  mImageWidth = 640;
  mImageHeight = 480;
  
  mDrawImageProgram = -1;
}


void DrawImage::init()
{
#ifdef VC_QOPENGL_FUNCTIONS
		initializeOpenGLFunctions();
#endif
		qDebug() << " vao : "<<mVAO<<endl;

		GL_CHECK( glBindVertexArray( mVAO ) );

		GL_CHECK( glGenBuffers( 2 , mVBO ) );
		
		GL_CHECK( glActiveTexture( GL_TEXTURE0 ) );

		GL_CHECK( glGenTextures( 1 , &mTexture ) );
		
		
		
		GLuint quad[  ] = { 0 , 3 , 1 , 3 , 2 , 1 };
		
		GLfloat quadVerts[]  = { 
		                         0 , 0 , 0.1 , 0 , 0 , 
		                         mImageWidth , 0 , 0.1 , 1 , 0 ,
				                 mImageWidth , mImageHeight , 0.1 , 1 , 1 ,
				                 0 , mImageHeight , 0.1  , 0 , 1 
		                       };
		
		GL_CHECK( glBindBuffer( GL_ARRAY_BUFFER , mVBO[ 0 ] ) );	   
		GL_CHECK( glBufferData( GL_ARRAY_BUFFER , 20 * sizeof( float ) , quadVerts , GL_STATIC_DRAW ) );

		GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , mVBO[ 1 ] ) );
		GL_CHECK( glBufferData( GL_ELEMENT_ARRAY_BUFFER , 6 * sizeof( GLuint ) , quad , GL_STATIC_DRAW ) );
		
		GL_CHECK( glEnableVertexAttribArray(0) );
		GL_CHECK( glEnableVertexAttribArray(1) );
		
		GL_CHECK( glVertexAttribPointer( 0 , 3 , GL_FLOAT, GL_FALSE, 5 * sizeof ( GLfloat )  , 0) );
		
		int offset = 3 * sizeof( GLfloat );

                GL_CHECK( glVertexAttribPointer( 1 , 2 , GL_FLOAT, GL_FALSE, 5 * sizeof ( GLfloat )  , ( float * )offset ) );
		
		GL_CHECK( glBindBuffer( GL_ARRAY_BUFFER , 0 ) );
		GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , 0 ) );
    
                GL_CHECK( glBindTexture(GL_TEXTURE_2D, mTexture) );
		
		GL_CHECK( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) );
	        GL_CHECK( glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR) ); 
//                 GL_CHECK( glActiveTexture( GL_TEXTURE0 ) );
		
		GL_CHECK( glBindVertexArray( 0 ) );
}


void DrawImage::render()
{
    GL_CHECK( glUseProgram( mDrawImageProgram ) );

    GL_CHECK( glBindVertexArray( mVAO ) );

    GL_CHECK( glBindBuffer(GL_ARRAY_BUFFER, mVBO[0]) );
    GL_CHECK( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mVBO[1] ) );


    GL_CHECK( glActiveTexture( GL_TEXTURE0 ) );

    GL_CHECK( glBindTexture( GL_TEXTURE_2D , mTexture ) );

    int texLoc = glGetUniformLocation( mDrawImageProgram , "ReferenceTexture" );

    GL_CHECK( glUniform1i( texLoc , 0 ) );
    
    GLint orthoMatrix = glGetUniformLocation( mDrawImageProgram , "orthoMatrix");
    
    GL_CHECK( glUniformMatrix4fv( orthoMatrix , 1 , false , mOrthoMatrix.data() ) );
    
    GL_CHECK( glDrawElements( GL_TRIANGLES , 6 , GL_UNSIGNED_INT , 0 ) );
    
    GL_CHECK( glBindBuffer(GL_ARRAY_BUFFER,  0 ) );
    GL_CHECK( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0 ) );
    
    GL_CHECK( glBindVertexArray( 0 ) );
     
    GL_CHECK( glUseProgram( 0 ) );

}

int DrawImage::addShaders(QString vertexShaderSrc, QString fragmentShaderSrc)
{
  		ShaderSource shaderSource;

		shaderSource.mVertexShaderSrc = vertexShaderSrc;
		shaderSource.mFragmentShaderSrc = fragmentShaderSrc;

		shaderSource.mHasGeometryShaderSrc = false;

		mShaderSources.push_back( shaderSource );

		GLuint vertexShader = compileShaders( GL_VERTEX_SHADER , vertexShaderSrc.toStdString().c_str() );
		GLuint fragmentShader = compileShaders( GL_FRAGMENT_SHADER, fragmentShaderSrc.toStdString().c_str() );


		GLuint program = glCreateProgram();

		GL_CHECK( glAttachShader( program , vertexShader ) );
		GL_CHECK( glAttachShader( program , fragmentShader) );

		mShaderPrograms.push_back( program );  
		
	        GL_CHECK( glBindAttribLocation( program , 0 , "position") );
                GL_CHECK( glBindAttribLocation( program , 1 , "texCoords") );

                GL_CHECK( glLinkProgram( program ) );

                GLint status;


                GL_CHECK( glGetProgramiv( program, GL_LINK_STATUS, &status ) );

                if( status == GL_FALSE )
                {
                   GLchar emsg[1024];
                   GL_CHECK( glGetProgramInfoLog( program , sizeof( emsg ) , 0 , emsg ) );
                   fprintf(stderr, "Error linking GLSL program : %s\n", emsg );
                   return -1;
                }
                
                mDrawImageProgram = program;

		return ( mShaderPrograms.size() - 1 );
}



void DrawImage::setImageDimensions(int w, int h)
{
  
  if( mImageWidth != w || mImageHeight != h )
  {
    
       
    
        mImageWidth = w;
        mImageHeight = h;
    
    		GLfloat quadVerts[]  = { 
		                         0 , 0 , 0.1 , 0 , 0 , 
		                         mImageWidth , 0 , 0.1 , 1 , 0 ,
				         mImageWidth , mImageHeight , 0.1 , 1 , 1 ,
				         0 , mImageHeight , 0.1  , 0 , 1 
		                       };
				       
				       
	GL_CHECK( glBindVertexArray( mVAO ) );			       
				       
				       
	GL_CHECK( glBindBuffer( GL_ARRAY_BUFFER , mVBO[ 0 ] ) );	   
	GL_CHECK( glBufferData( GL_ARRAY_BUFFER , 20 * sizeof( float ) , quadVerts , GL_STATIC_DRAW ) );
		
	GL_CHECK( glBindBuffer( GL_ARRAY_BUFFER , 0 ) );
		
	GL_CHECK( glBindVertexArray( 0 ) );
  }
  

  
  mOrthoMatrix.setToIdentity();
  
  mOrthoMatrix.ortho( 0 , mImageWidth  , mImageHeight , 0, -1.0f , 1.0f );
  
}


void DrawImage::setImage(cv::Mat& image)
{
   
   mCurrentImage = image;
  
   mIsImageChanged = true;
   
}


void DrawImage::updateImage()
{
   setImageDimensions( mCurrentImage.cols , mCurrentImage.rows );

   GL_CHECK( glBindVertexArray( mVAO ) );
   
   GL_CHECK( glBindTexture( GL_TEXTURE_2D , mTexture ) );

   if( mCurrentImage.channels() == 4 )
   {
     GL_CHECK( glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, mImageWidth , mImageHeight , 0, GL_BGRA, GL_UNSIGNED_BYTE , mCurrentImage.data ) );
   }
   else
   {
     GL_CHECK( glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, mImageWidth , mImageHeight , 0, GL_RGB, GL_UNSIGNED_BYTE , mCurrentImage.data ) );
   }
   

   GL_CHECK( glBindTexture( GL_TEXTURE_2D , 0 ) );
   
   GL_CHECK( glBindVertexArray( 0 ) );
  
  
}




void DrawImage::setVAO( GLuint vao )
{
  mVAO = vao;
}


void DrawImage::setViewPort( int w, int h )
{
  GL_CHECK( glViewport( 0 , 0 , w , h ) );
}


DrawImage::~DrawImage()
{

}


}

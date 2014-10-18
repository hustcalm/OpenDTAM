#ifndef DRAWIMAGE_H
#define DRAWIMAGE_H


#include "Visualization3D/buffermanager.h"
#include "QMatrix4x4"
#include "external_includes/opencvincludes.h"

namespace vc{

class DrawImage :  public BufferManager
{
  

public:
  
  
  struct ShaderSource{
  
  QString mVertexShaderSrc , mFragmentShaderSrc , mGeometryShaderSrc;
  
  bool mHasGeometryShaderSrc;
  
};  

  
    
  DrawImage();
  
  void setVAO( GLuint vao );
  
  void generateVertexArray();
  
  void init();
  void render();
  void setViewPort( int w , int h );
  void setImageDimensions( int w , int h );
  void setImage( cv::Mat& image );
  
  int addShaders( QString vertexShaderSrc , QString fragmentShaderSrc );
    
    
  ~DrawImage();
  
  
protected:
  
  void readShaderFromFile( QString fileName , QString& shaderText );
  
  void updateImage();

  GLuint mVAO , mTexture , mVBO[ 2 ];
  
  QMatrix4x4 mOrthoMatrix;
  
  int mImageWidth , mImageHeight;
  
  std::vector< ShaderSource > mShaderSources;
  std::vector< GLint > mShaderPrograms;
  
  GLint mDrawImageProgram;
  
  cv::Mat mCurrentImage;
  
  bool mIsImageChanged;
  
  ShaderSource mShaderSource ;
  
};

}

#endif // DRAWIMAGE_H

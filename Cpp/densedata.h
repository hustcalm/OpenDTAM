/*
 * Copyright 2014 <copyright holder> <email>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#ifndef DENSEDATA_H
#define DENSEDATA_H

#include "external_includes/openglincludes.h"
#include "Visualization3D/trackballcamera.h"
#include "Visualization3D/buffermanager.h"
#include "external_includes/eigenincludes.h"
#include "QString"
namespace vc{

class DenseData : public vc::BufferManager
{
  
public:
  
  

    
struct VertexData
{
    
    
    Eigen::Vector3f mVertex ;
    
    Eigen::Vector3f mColor;
    
    Eigen::Vector3f mNormal ;
  
} ;
  
    DenseData();
    ~DenseData();
  
  
  struct ShaderSource{
  
  QString mVertexShaderSrc , mFragmentShaderSrc , mGeometryShaderSrc;
  
  bool mHasGeometryShaderSrc;
  
};  
    
    
  int addShaders( QString vertexShaderSrc , QString geometryShaderSrc , QString fragmentShaderSrc );
  int addShaders( QString vertexShaderSrc , QString fragmentShaderSrc );

  int addDebugShaders(  QString vertexShaderSrc , QString geometryShaderSrc , QString fragmentShaderSrc  );
  
  void init();
  
  void resetCamera();
  
  void setCameraToInitialized();
  
  void generateVertexArrays();
  
  void setVAO( const GLuint &vao );
  
  void render();
  
  int getNumVertices() ;
  int getNumFaces() ;
  
  void setData( std::vector< Eigen::Vector3f > &vertices , std::vector< Eigen::Vector3f > &colors  );
  void setData( std::vector< Eigen::Vector3f > &vertices , std::vector< Eigen::Vector3f > &colors , std::vector< GLuint > &indices );
  
  void setCamera( TrackBallCamera *camera );
  
  Eigen::Vector3f getObjectCenter();
  
  float getRadius();
  
  void getCage( std::vector< Eigen::Vector3f > &cage );
  
  void setDisplayModeToPointCloud();
  void setDisplayModeToPointSurface();
  void setDisplayModeToWireFrame();
  void setDisplayModeToWireframeOverlay();
  
  void setPointCloudProgram( int programId );
  void setWireframeProgram( int programId );
  void setSurfaceProgram( int programId );
  
protected:

void buildDimensions();  
  
  
protected:

  GLuint mVAO , mVBO[ 4 ] ;
  
  GLint mDebugProgram , mNumTriangles;
  
  std::vector< VertexData > mVertexData;
  
  std::vector< unsigned int > mShaderPrograms;
  
  std::vector< ShaderSource > mShaderSources;
  
  GLuint mPointCloudProgram , mSurfaceProgram , mWireframeProgram;
  
  std::vector< GLuint > mFaceIndices , mVertexIndices , mWireframeIndices;
  
  bool mBuffersInitialized , mHasNewData;
  
  float mRadius;
  
  Eigen::Vector3f mObjectCenter;
  
  bool mIsCameraInitialized;
  
  std::vector< Eigen::Vector3f > mCage;
  
  TrackBallCamera *mCamera;
  
  enum DisplayModes{ POINT_CLOUD = 0 , SURFACE , WIREFRAME , WIREFRAME_OVERLAY  } mDisplayModes;
  
  ShaderSource mPointCloudShaderSource , mSurfaceShaderSource , mWireframeShaderSource;
  
  
};

}

#endif // DENSEDATA_H

#include "densedata.h"
#include "Visualization3D/openglhelper.h"

#include "QFile"
#include "QFileInfo"
#include "QDebug"
#include "QTextStream"

#include "iostream"

#ifdef VC_WINDOWS

#undef min
#undef max

#endif


namespace vc{
  
  
void readShaderFromFile( QString fileName , QString& shaderText )
{
	 QFile file( fileName );

	 QFileInfo fileInfo( file );

	 if (!file.open(QIODevice::ReadOnly))
     {
         qDebug() << "Cannot open file for reading: "
                 << qPrintable(file.errorString()) << endl;
         return;
      }

	QTextStream reader( &file );

	shaderText = reader.readAll();
}  
  

DenseData::DenseData() 
{
  mBuffersInitialized = false;

  mCamera = 0;
  
  mNumTriangles = 0;
  
    
 readShaderFromFile( ":/densevert.glsl" , mPointCloudShaderSource.mVertexShaderSrc ); 
 readShaderFromFile( ":/densepointcloudfrag.glsl" , mPointCloudShaderSource.mFragmentShaderSrc );
 
 mDisplayModes = POINT_CLOUD;
 
//  qDebug() << mPointCloudShaderSource.mFragmentShaderSrc << endl;
    
#ifdef VC_QOPENGL_FUNCTIONS
  initializeOpenGLFunctions();
#endif  
}



void DenseData::buildDimensions()
{
    float xMax = std::numeric_limits<float>::min() , xMin = std::numeric_limits<float>::max();
    float yMax = std::numeric_limits<float>::min() , yMin = std::numeric_limits<float>::max();
    float zMax = std::numeric_limits<float>::min() , zMin = std::numeric_limits<float>::max();
    
    int numVertices = mVertexData.size();
    
    for( int ii = 0; ii < numVertices ; ii++ )
    {
        xMax = std::max( xMax , mVertexData[ ii ].mVertex( 0 ) );
        xMin = std::min( xMin , mVertexData[ ii ].mVertex( 0 ) );

        yMax = std::max( yMax , mVertexData[ ii ].mVertex( 1 ) );
        yMin = std::min( yMin , mVertexData[ ii ].mVertex( 1 ) );

        zMax = std::max( zMax , mVertexData[ ii ].mVertex( 2 ) );
        zMin = std::min( zMin , mVertexData[ ii ].mVertex( 2 ) );

//         std::cout << mVertexData[ ii ].mVertex.transpose() << std::endl;
    }

    mRadius = ( xMax - xMin ) * ( xMax - xMin ) + ( yMax - yMin ) * ( yMax - yMin ) + ( zMax - zMin ) * ( zMax - zMin ) ;

    mRadius = sqrt( mRadius ) / 2;

    mObjectCenter( 0 ) = ( xMax + xMin ) / 2;
    mObjectCenter( 1 ) = ( yMax + yMin ) / 2;
    mObjectCenter( 2 ) = ( zMax + zMin ) / 2;

    mCage.resize( 8 );

    mCage[ 0 ] = Eigen::Vector3f( xMin , yMin , zMin );
    mCage[ 1 ] = Eigen::Vector3f( xMax , yMin , zMin );
    mCage[ 2 ] = Eigen::Vector3f( xMax , yMax , zMin );
    mCage[ 3 ] = Eigen::Vector3f( xMin , yMax , zMin );

    mCage[ 4 ] = Eigen::Vector3f( xMin , yMin , zMax );
    mCage[ 5 ] = Eigen::Vector3f( xMax , yMin , zMax );
    mCage[ 6 ] = Eigen::Vector3f( xMax , yMax , zMax );
    mCage[ 7 ] = Eigen::Vector3f( xMin , yMax , zMax );
}


int DenseData::addDebugShaders(  QString vertexShaderSrc , QString geometryShaderSrc , QString fragmentShaderSrc  )
{
    ShaderSource shaderSource;
    
    shaderSource.mVertexShaderSrc = vertexShaderSrc;
    shaderSource.mFragmentShaderSrc = fragmentShaderSrc;
    shaderSource.mGeometryShaderSrc = geometryShaderSrc;
    
    shaderSource.mHasGeometryShaderSrc = true;
    
    mShaderSources.push_back( shaderSource );
    
    GLuint vertexShader = compileShader( GL_VERTEX_SHADER , vertexShaderSrc.toStdString().c_str() );
    GLuint geometryShader = compileShader( GL_GEOMETRY_SHADER, geometryShaderSrc.toStdString().c_str() );
    GLuint fragmentShader = compileShader( GL_FRAGMENT_SHADER, fragmentShaderSrc.toStdString().c_str() );
    
    
    GLuint program = glCreateProgram();
    
    GL_CHECK( glBindAttribLocation( program , 0 , "position") );
    GL_CHECK( glBindAttribLocation( program , 1 , "color") );
    GL_CHECK( glBindAttribLocation( program , 2 , "normal") );
    
    GL_CHECK( glAttachShader( program , vertexShader ) );
    GL_CHECK( glAttachShader( program , geometryShader) );
    GL_CHECK( glAttachShader( program , fragmentShader) );
    
    mShaderPrograms.push_back( program );
    
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
    
    int setId = mShaderPrograms.size() - 1;

    mDebugProgram = mShaderPrograms[ setId ];

    return setId;
}


    
  int DenseData::addShaders( QString vertexShaderSrc , QString geometryShaderSrc , QString fragmentShaderSrc )
  {
    
    return -1;
  }
  
  
  int DenseData::addShaders( QString vertexShaderSrc , QString fragmentShaderSrc )
  {
    
    ShaderSource shaderSource;
    
    shaderSource.mVertexShaderSrc = vertexShaderSrc;
    shaderSource.mFragmentShaderSrc = fragmentShaderSrc;
    
    shaderSource.mHasGeometryShaderSrc = false;
    
    mShaderSources.push_back( shaderSource );
    
    GLuint vertexShader = compileShader( GL_VERTEX_SHADER , vertexShaderSrc.toStdString().c_str() );
    GLuint fragmentShader = compileShader( GL_FRAGMENT_SHADER, fragmentShaderSrc.toStdString().c_str() );
    
    
    GLuint program = glCreateProgram();
    
    GL_CHECK( glBindAttribLocation( program , 0 , "position") );
    GL_CHECK( glBindAttribLocation( program , 1 , "color") );
    GL_CHECK( glBindAttribLocation( program , 2 , "normal") );
    
    GL_CHECK( glAttachShader( program , vertexShader ) );
    GL_CHECK( glAttachShader( program , fragmentShader) );
    
    mShaderPrograms.push_back( program );
    
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
    
    int setId = mShaderPrograms.size() - 1;

    mDebugProgram = mShaderPrograms[ setId ];


    return setId;
  }
  
  
void DenseData::generateVertexArrays()
{
  GL_CHECK( glGenVertexArrays( 1 , &mVAO ) );
}


void DenseData::getCage(std::vector< Eigen::Vector3f >& cage)
{
  cage = mCage;
}


void DenseData::setDisplayModeToPointCloud()
{
  mDisplayModes = POINT_CLOUD;  
}

void DenseData::setDisplayModeToPointSurface()
{
  mDisplayModes = SURFACE;
}

void DenseData::setDisplayModeToWireFrame()
{
  mDisplayModes = WIREFRAME;
}

void DenseData::setDisplayModeToWireframeOverlay()
{
  mDisplayModes = WIREFRAME_OVERLAY;
}


void DenseData::setPointCloudProgram( int programId )
{
    mPointCloudProgram = mShaderPrograms[ programId ];
}

void DenseData::setWireframeProgram( int programId )
{
  
  mWireframeProgram = mShaderPrograms[ programId ];
}
  
void DenseData::setSurfaceProgram( int programId )
{
  mSurfaceProgram = mShaderPrograms[ programId ];
}


int DenseData::getNumFaces()
{
  return mNumTriangles;
}

int DenseData::getNumVertices()
{
  return mVertexData.size();
}

Eigen::Vector3f DenseData::getObjectCenter()
{
  return mObjectCenter;
}

float DenseData::getRadius()
{
  return mRadius;
}

void DenseData::init()
{
  
    mIsCameraInitialized = false; 
  
    int shaderId =  addShaders( mPointCloudShaderSource.mVertexShaderSrc , mPointCloudShaderSource.mFragmentShaderSrc );
    
    setPointCloudProgram( shaderId );
  
    GL_CHECK( glBindVertexArray( mVAO ) );

    GL_CHECK( glUseProgram( mDebugProgram ) );
    
    GL_CHECK( glGenBuffers( 4 , mVBO ) );
    
    GL_CHECK( glBindBuffer( GL_ARRAY_BUFFER , mVBO[ 0 ] ) );

    GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , mVBO[ 1 ] ) );
    
    GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , mVBO[ 2 ] ) );
    
    GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , mVBO[ 3 ] ) );
    

    int offset = 0;

    GL_CHECK( glVertexAttribPointer( 0 , 3, GL_FLOAT, GL_FALSE, sizeof( VertexData )  , 0) );

    offset += 3 * sizeof( GLfloat );

    GL_CHECK( glVertexAttribPointer( 1 , 3 , GL_FLOAT, GL_FALSE, sizeof (  VertexData  ) , ( float * )offset ) );
    
    offset += 3 * sizeof( GLfloat );
    
    GL_CHECK( glVertexAttribPointer( 2 , 3 , GL_FLOAT, GL_FALSE, sizeof (  VertexData  ) , ( float * )offset ) );
    
        
    GL_CHECK( glEnableVertexAttribArray(0) );
    GL_CHECK( glEnableVertexAttribArray(1) );
    GL_CHECK( glEnableVertexAttribArray(2) );
    
    
    GL_CHECK( glBindBuffer( GL_ARRAY_BUFFER , 0 ) );
    GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , 0 ) );
    
    

    GL_CHECK( glUseProgram( 0) );
    
    GL_CHECK( glBindVertexArray(0) );
}

void DenseData::render()
{
    GL_CHECK( glBindVertexArray(mVAO) );
  
  
    if( mHasNewData )
    {
      //update vertices
      GL_CHECK( glBindBuffer(GL_ARRAY_BUFFER, mVBO[ 0 ]) );
      GL_CHECK( glBufferData( GL_ARRAY_BUFFER , mVertexData.size() * sizeof( VertexData ) , mVertexData.data() , GL_DYNAMIC_DRAW ) );
      //update points
      GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , mVBO[ 1 ]) );
      GL_CHECK( glBufferData( GL_ELEMENT_ARRAY_BUFFER , mVertexIndices.size() * sizeof( GLuint ) , mVertexIndices.data() , GL_DYNAMIC_DRAW ) );
      //update faces 
      GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , mVBO[ 2 ]) );
      GL_CHECK( glBufferData( GL_ELEMENT_ARRAY_BUFFER , mFaceIndices.size() * sizeof( GLuint ) , mFaceIndices.data() , GL_DYNAMIC_DRAW ) );
      //update edges
      GL_CHECK( glBindBuffer( GL_ELEMENT_ARRAY_BUFFER , mVBO[ 3 ]) );
      GL_CHECK( glBufferData( GL_ELEMENT_ARRAY_BUFFER , mWireframeIndices.size() * sizeof( GLuint ) , mWireframeIndices.data() , GL_DYNAMIC_DRAW ) );
      
      mHasNewData = false;
    }

    
    GL_CHECK( glBindBuffer(GL_ARRAY_BUFFER, mVBO[ 0 ]) );
    
    
    
    if( mDisplayModes == POINT_CLOUD )
    {
        GL_CHECK( glUseProgram( mPointCloudProgram ) );
      
        QMatrix4x4 mv = mCamera->getModelViewMatrix();
        QMatrix4x4 mat1 = mCamera->getModelViewProjectionMatrix();
        Eigen::Matrix3f mat2 = mCamera->getNormalTransformationMatrix();
	
	GL_CHECK( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mVBO[ 1 ]) );
    
	GLint mvpMatrix = glGetUniformLocation( mPointCloudProgram , "mvpMatrix");

        GL_CHECK( glUniformMatrix4fv( mvpMatrix , 1 , false , mat1.data() ) );
    
        GL_CHECK( glDrawElements( GL_POINTS , mFaceIndices.size() , GL_UNSIGNED_INT , 0 ) );
    }
    else if( mDisplayModes == SURFACE )
    {
         GL_CHECK( glUseProgram( mSurfaceProgram ) );
      
        QMatrix4x4 mv = mCamera->getModelViewMatrix();
        QMatrix4x4 mat1 = mCamera->getModelViewProjectionMatrix();
        Eigen::Matrix3f mat2 = mCamera->getNormalTransformationMatrix();
	
	GL_CHECK( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mVBO[ 2 ]) );
    
	GLint mvpMatrix = glGetUniformLocation( mSurfaceProgram , "mvpMatrix");

        GL_CHECK( glUniformMatrix4fv( mvpMatrix , 1 , false , mat1.data() ) );
    
        GL_CHECK( glDrawElements( GL_TRIANGLES , mFaceIndices.size() , GL_UNSIGNED_INT , 0 ) );
    }
    else if( mDisplayModes == WIREFRAME )
    {
         GL_CHECK( glUseProgram( mWireframeProgram ) );
      
        QMatrix4x4 mv = mCamera->getModelViewMatrix();
        QMatrix4x4 mat1 = mCamera->getModelViewProjectionMatrix();
        Eigen::Matrix3f mat2 = mCamera->getNormalTransformationMatrix();
	
	GL_CHECK( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mVBO[ 3 ]) );
    
	GLint mvpMatrix = glGetUniformLocation( mWireframeProgram , "mvpMatrix");

        GL_CHECK( glUniformMatrix4fv( mvpMatrix , 1 , false , mat1.data() ) );
    
        GL_CHECK( glDrawElements( GL_LINES , mFaceIndices.size() , GL_UNSIGNED_INT , 0 ) );
    }
    else
    {
        GL_CHECK( glUseProgram( mSurfaceProgram ) );
      
        QMatrix4x4 mv = mCamera->getModelViewMatrix();
        QMatrix4x4 mat1 = mCamera->getModelViewProjectionMatrix();
        Eigen::Matrix3f mat2 = mCamera->getNormalTransformationMatrix();
	
	GL_CHECK( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mVBO[ 2 ]) );
    
	GLint mvpMatrix = glGetUniformLocation( mSurfaceProgram , "mvpMatrix");

        GL_CHECK( glUniformMatrix4fv( mvpMatrix , 1 , false , mat1.data() ) );
    
        GL_CHECK( glDrawElements( GL_TRIANGLES , mFaceIndices.size() , GL_UNSIGNED_INT , 0 ) );
      
    }

    GL_CHECK( glBindBuffer( GL_ARRAY_BUFFER, 0 ) );
    GL_CHECK( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0) );
    
    GL_CHECK( glBindVertexArray( 0 ) );
	
    GL_CHECK( glUseProgram( 0 ) );
}

void DenseData::setCamera(TrackBallCamera* camera)
{
  mCamera = camera;
}

void DenseData::setVAO(const GLuint& vao)
{
  mVAO = vao;
}

void DenseData::resetCamera()
{
  
  if( mCamera )
  {
      mCamera->setObjectCenter( QVector3D( mObjectCenter.x() , mObjectCenter.y() , mObjectCenter.z() ) );
      mCamera->setRadius( mRadius );
      mCamera->init();
      
      mIsCameraInitialized = true;
  }
}


  void DenseData::setData( std::vector< Eigen::Vector3f > &vertices , std::vector< Eigen::Vector3f > &colors  )
  {
    mVertexData.resize( vertices.size() );
    
    int numVertices = vertices.size();
    
    mVertexIndices.resize( numVertices );
    
    for( int vv = 0; vv < numVertices ; vv++ )
    {
      mVertexData[ vv ].mVertex = vertices[ vv ];
      mVertexData[ vv ].mColor = colors[ vv ];
      
       mVertexIndices[ vv ] = vv;
    }
    
    buildDimensions();
    
    if( !mIsCameraInitialized && mCamera )
    {
      mCamera->setObjectCenter( QVector3D( mObjectCenter.x() , mObjectCenter.y() , mObjectCenter.z() ) );
      mCamera->setRadius( mRadius );
      mCamera->init();
      
      mIsCameraInitialized = true;
      
      qDebug() << " initializing camera  "<<endl;
    }
//     else
//     {
//       qDebug() << " camera not initialized  "<<endl;
//     }

    mHasNewData = true;
    
  }
  
  
  void DenseData::setData( std::vector< Eigen::Vector3f > &vertices , std::vector< Eigen::Vector3f > &colors , std::vector< GLuint > &indices )
  {
    
    mVertexData.resize( vertices.size() );
    
    int numVertices = vertices.size();
    
    mVertexIndices.resize( numVertices );
    
        
    for( int vv = 0; vv < numVertices ; vv++ )
    {
      mVertexData[ vv ].mVertex = vertices[ vv ];
      mVertexData[ vv ].mColor = colors[ vv ];
      
      mVertexIndices[ vv ] = vv;
    }
    
    mFaceIndices = indices;
    
    buildDimensions();
    
    if( !mIsCameraInitialized && mCamera )
    {
      mCamera->setObjectCenter( QVector3D( mObjectCenter.x() , mObjectCenter.y() , mObjectCenter.z() ) );
      mCamera->setRadius( mRadius );
      mCamera->init();
      
      mIsCameraInitialized = true;
      
      qDebug() << " initializing camera  "<<endl;
    }
    
     mHasNewData = true;
  }




DenseData::~DenseData()
{

}


}

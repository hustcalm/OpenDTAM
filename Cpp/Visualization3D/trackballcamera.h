#ifndef __TRACKBALL_CAMERA_H__
#define __TRACKBALL_CAMERA_H__

#ifdef VC_WINDOWS
#include "windows.h"
#endif

#include "openglcamera.h"
#include "QMatrix4x4"
#include "external_includes/eigenincludes.h"


class TrackBallCamera : public OpenGLCamera
{
  
  QMatrix4x4 mModelView , mProjectionMatrix , mRotation , mTranslation;

  Eigen::Matrix4f mEModelView , mEProjectionMatrix , mERotation , mEtranslation;
  
  QVector3D mObjectCenter , mLastPoint3D;
  
  float mRadius;
  
  unsigned int mViewPortWidth , mViewPortHeight;
  
  bool mLastPointOK;
  
  int mWidth , mHeight;

  bool mStartTrackBall;

  QVector3D mRotationTarget;

  std::vector< Eigen::Vector3f > mCorners;

  QPoint mLastPoint;
  
  QPointF mCurrentMousePos;
  
  int mCurrentKey , mCurrentMouseButton;
  
  
protected:

 bool mapToSphere( QPoint pos , QVector3D &mappedPoint );  

 void viewAll();

 

 void intersectWithTrackBallSphere( QVector3D &origin , QVector3D &ray , QVector3D &intersectionPoint );

 void rotate( float angle , QVector3D axis );

 void translate( const Eigen::Vector3f &translationVector );

 void checkCenter();

    
  
public:
  
  TrackBallCamera();
  
  
  QMatrix4x4 getModelViewMatrix();
  QMatrix4x4 getModelViewProjectionMatrix();  

  Eigen::Matrix3f getNormalTransformationMatrix();

  void updateProjectionMatrix( int w , int h);
  
  void registerMousePress( const QPoint& pos  , const Qt::MouseButtons &buttons  );
  void registerMouseMove( const QPoint& pos  , const Qt::MouseButtons &buttons );
  void registerMouseRelease( const QPoint& pos );
  
  void registerMousePos( const QPoint& pos );

  void registerMouseWheel( int delta );
  
  void registerKeyPress( int key );
  void registerKeyRelease( int key );
  
  void setObjectCenter( QVector3D objectCenter );

  void setRadius( float radius );
  
  void setViewPortDimension( unsigned int w , unsigned int h );

  void getWinScale( float &wScale , float &hScale );

  virtual void getModelViewMatrix( double *mv );
  virtual void getProjectionMatrix( double *p );

  virtual void getRay( QPointF pos , Eigen::Vector3f &origin , Eigen::Vector3f &dir );
  
  void getRay( Eigen::Vector3f &origin , Eigen::Vector3f &dir );
  
  float radius( float R, Eigen::Vector3f& intersection );
  
  void track( const QPoint& pos , const Qt::MouseButtons &buttons );

  void init();


  void testSetCageCorners( std::vector< Eigen::Vector3f > &corners ); 
  
  int getCurrentKey();
  int getCurrentMouseButton();
  
  
};


#endif

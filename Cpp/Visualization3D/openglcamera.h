#ifndef __OPENGL_CAMERA_H__
#define __OPENGL_CAMERA_H__


#include "QMatrix4x4"
#include "QPoint"
#include "external_includes/eigenincludes.h"

class OpenGLCamera{
  
  
  
public:
  
  OpenGLCamera();

  virtual void registerMousePress( const QPoint &pos  , const Qt::MouseButtons &buttons   ) = 0;
  virtual void registerMouseMove( const QPoint &pos , const Qt::MouseButtons &buttons ) = 0;
  virtual void registerMouseRelease( const QPoint &pos ) = 0;

  virtual void registerMouseWheel( const int delta ) = 0;
  
  virtual void registerMousePos( const QPoint& pos ) = 0;
  
  virtual void registerKeyPress( const int key ) = 0;
  virtual void registerKeyRelease( const int key ) = 0;  
  
  virtual QMatrix4x4 getModelViewMatrix() = 0; 
  virtual QMatrix4x4 getModelViewProjectionMatrix() = 0; 
  virtual Eigen::Matrix3f getNormalTransformationMatrix() = 0; 

  virtual void setViewPortDimension( unsigned int w , unsigned int h ) = 0;

  virtual void getModelViewMatrix( double *mv ) = 0;
  virtual void getProjectionMatrix( double *p ) = 0;

  virtual void updateProjectionMatrix( int w , int h) = 0;

  virtual void getWinScale( float &wScale , float &hScale ) = 0;
  
  virtual void getRay( QPointF pos , Eigen::Vector3f &origin , Eigen::Vector3f &dir ) = 0;
   
  virtual void getRay( Eigen::Vector3f &origin , Eigen::Vector3f &dir ) = 0;
   
  virtual float radius(  float R, Eigen::Vector3f& intersection ) = 0;
  
  virtual int getCurrentKey() = 0;
  virtual int getCurrentMouseButton() = 0;
  
};


#endif

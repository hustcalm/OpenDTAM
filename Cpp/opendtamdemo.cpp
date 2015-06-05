#include "external_includes/openglincludes.h"
#include <opencv2/core/core.hpp>
#include "external_includes/eigenincludes.h"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>

// Mine
#include "convertAhandaPovRayToStandard.h"
#include "CostVolume/utils/reproject.hpp"
#include "CostVolume/utils/reprojectCloud.hpp"
#include "CostVolume/Cost.h"
#include "CostVolume/CostVolume.hpp"
#include "Optimizer/Optimizer.hpp"
#include "DepthmapDenoiseWeightedHuber/DepthmapDenoiseWeightedHuber.hpp"
#include "graphics.hpp"
#include "set_affinity.h"

#include "utils/utils.hpp"

// Debug
#include "tictoc.h"

#include "QtConcurrent/QtConcurrent"

#include "ptam/ATANCamera.h"
#include "ptam/MapMaker.h"
#include "ptam/MapViewer.h"
#include "ptam/Tracker.h"
#include "ptam/Map.h"
#include "ptam/MapPoint.h"
#include "ptam/CalibImage.h"
#include "ptam/CalibCornerPatch.h"

#include "Visualization3D/openglhelper.h"
#include <GLFW/glfw3.h>
#include "Visualization3D/trackballcamera.h"
#include "depthmapstereo.h"
#include "projectdata.h"
#include "densedata.h"
#include "drawimage.h"

#include "QThread"
#include <pthread.h>

#define DTAM_LAYERS 32

using namespace cv;
using namespace cv::gpu;
using namespace std;

GLFWwindow* g_window = 0;
GLFWmonitor* g_primary = 0;

ProjectData *g_projectData;

Eigen::Matrix4f g_currentPose;
Eigen::Matrix3f g_intrinsics;
Eigen::Matrix< float , 5 , 1  > g_ptamCamParams;

bool g_runDTAM = false , g_NewKeyFrame = false;

float g_minDistance = 1.0 , g_maxDistance = 1.0;

int g_computeDepthMap = 0;

int   g_width = 1024,
      g_height = 760;
      
static bool g_running = true;      
int g_prev_x = 0 , g_prev_y = 0 , g_frame = 0 ;  

int g_spaceBarPressed = 0;

TrackBallCamera *g_camera = 0 , *g_ptamCamera = 0 , *g_OpenCVStereoDenseCamera = 0; 

vc::DenseData *g_mesh = 0 , *g_ptamSparseData = 0 , *g_DTAMDenseData = 0 , *g_OpenCVStereoDenseData = 0;
vc::DrawImage *g_ptamDrawImage = 0 , *g_disparityImage = 0;

enum DisplayModes{ PTAM_IMAGE = 0 , PTAM_SPARSE , DTAM_DENSE , OPENCV_STEREO_DENSE , STEREO_DISPARITY } g_displayModes;

cv::Mat g_currentImage;

bool g_ptamInitialized = false;

GLuint g_VAO;

int g_mbutton[3] = {0, 0, 0};

static void setGLCoreProfile()
{
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
}

void display();
void update();

void init3dViewer();

class ViewerThread : public QThread
{
public:
  void run();
};

class MainThread : public QThread
{
public:
  int argc;
  char **argv;
  
  void run();
};

// GLFW
void reshape(GLFWwindow *, int width, int height) 
{

    g_width = width;
    g_height = height;
    
    GL_CHECK( glViewport( 0 , 0 , g_width , g_height ) );
    
    g_camera->setViewPortDimension( g_width , g_height );
    g_ptamCamera->setViewPortDimension( g_width , g_height );
    g_OpenCVStereoDenseCamera->setViewPortDimension( g_width , g_height );
}

void windowClose(GLFWwindow*) 
{
    g_running = false;
}

static void motion(GLFWwindow *, double dx, double dy);

static void mouse(GLFWwindow *, int button, int state, int mods);

void scroll(GLFWwindow* window,double x,double y);

// Idle is called between frames, here we advance the frame number and update the procedural animation that is being applied to the mesh
/*
void idle() 
{
    g_frame++;
    update();
}
*/

static void keyboard(GLFWwindow *, int key, int /* scancode */, int event, int /* mods */) ;

void setIntrinsicsPTAMType( float fx , float fy , float cx , float cy , float barrelDist );
void drawTrailPoints( std::vector< std::pair< QPoint , QPoint > > &points , cv::Mat &image );
void rawFastCorners( std::vector< QPoint > &points , cv::Mat &image );
void drawTrackedPoints( std::vector< QPoint > &trackedPoints , cv::Mat &image );
void runPtam( cv::Mat& image , ProjectData *projectData );
void grabFrame( cv::Mat& image , CVD::Image<CVD::Rgb<CVD::byte> > &coloredImage , CVD::Image< CVD::byte > &grayImage );
void ptam( Mat& frame );
void undistortImageBarrel( cv::Mat &image , cv::Mat &udImage , double dw , double dwInv , 
		            double d2Tan , double fx , double fy , double cx , double cy );
void displayImage( cv::Mat &image );

void computeRays( int w , int h , const Eigen::Matrix3f  &rotation ,const  Eigen::Vector3f &translation , Eigen::Matrix3f &K , cv::Mat &rays );
 
void computeAndDisplayPoints( Mat& rays, Mat& depths, Mat& colorFrame, Eigen::Vector3f& cameraCenter, CostVolume& cost );

void filterPoints( Eigen::Vector3f &cameraCenter , std::vector< Eigen::Vector3f > &points , std::vector< Eigen::Vector3f > &filteredPoints, float &minDistance , float &maxDistance );
 
void computeStereoCorresp();
 
void init3dViewer();

void ptam( cv::Mat &frame , Eigen::Matrix4f &pose );

int App_main( int argc, char** argv );

void myExit(){
    ImplThread::stopAllThreads();
}

const static bool valgrind = 0;

int main( int argc, char** argv )
{
    // For DSCF0195
    //setIntrinsicsPTAMType( 1.04464 , 1.3932 , 0.496628 , 0.512184 , 0.675691 );

    // For Logic Pro9000
    setIntrinsicsPTAMType( 0.845999, 1.12317, 0.511039, 0.458354, -0.169906);
  
    std::cout<<"New viewer thread..."<<std::endl;
    ViewerThread viewer;

    std::cout<<"Starting viewer thread..."<<std::endl;
    viewer.start();

    std::cout<<"New ProjectData..."<<std::endl;
    g_projectData = new ProjectData();

    std::cout<<"Bringing the GUI up..."<<std::endl;
    initGui();

    std::cout<<"Main thread..."<<std::endl;
    MainThread mt;
    
    mt.argc = argc;
    mt.argv = argv;
    
    std::cout<<"Starting main thread..."<<std::endl;
    mt.start();

    // Like Pthread join, let the main thread wait for the threads to finish
    mt.wait();
    viewer.wait();    
     
    myExit();
    
    return 0;
}

int App_main( int argc, char** argv )
{
    Mat image, cameraMatrix( 3 , 3 , CV_64FC1 ), R( 3 , 3 , CV_64FC1 ), T( 3 , 1 , CV_64FC1 );
    vector<Mat> images, Rs, Ts;
    Mat ret; // A place to return downloaded images to
    
    cv::VideoCapture cap;

    cv::Mat intrinsics( 3 , 3 , CV_64FC1 ) , d( 4 , 1 , CV_64FC1 );
    
    cv::setIdentity( intrinsics );
    cv::setIdentity( cameraMatrix );
    
    cameraMatrix.at< double >( 0 , 0 ) = g_intrinsics( 0 , 0 );
    cameraMatrix.at< double >( 1 , 1 ) = g_intrinsics( 1 , 1 );
    cameraMatrix.at< double >( 0 , 2 ) = g_intrinsics( 0 , 2 );
    cameraMatrix.at< double >( 1 , 2 ) = g_intrinsics( 1 , 2 );
    
    d.setTo( cv::Scalar( 0 ) );
     
    // DSCF0195
    cap.open(DATASET_PATH);
    
    // Logitec Pro9000
    //cap.open(DATASET_PATH_PRO9000);

    if( !cap.isOpened() )
    {
        std::cout << "Could not initialize capturing...\n";
	
        return 0;
    }
    else {
        std::cout << "Capture initilization is done!" << std::endl;
    }
    
    double reconstructionScale = 5/5.;

    CudaMem cret(480, 640, CV_32FC1);
    ret = cret.createMatHeader();
    
    // Setup camera matrix
    double sx = reconstructionScale;
    double sy = reconstructionScale;
    cameraMatrix += (Mat)(Mat_<double>(3,3) <<    0.0,0.0,0.5,
                                                0.0,0.0,0.5,
                                                0.0,0.0,0.0);
    cameraMatrix = cameraMatrix.mul((Mat)(Mat_<double>(3,3) <<    sx,0.0,sx,
                                                                0.0,sy ,sy,
                                                                0.0,0.0,1.0));
    cameraMatrix -= (Mat)(Mat_<double>(3,3) <<    0.0,0.0,0.5,
                                                0.0,0.0,0.5,
                                                0.0,0.0,0);
    int layers = DTAM_LAYERS; // Depth layers
    int imagesPerCV = 30; // Images used per cost volume
    CostVolume cv;

    int imageNum = 0;
    
    cv::Mat rays( 480 , 640 , CV_32FC3  );
   
    Eigen::Vector3f cameraCenter;
    
    cv::gpu::Stream s; // For GPU Async Tasks
     
    for(;;)
    {
        cv::Mat frame, backupFrame, fImage;
        cap >> frame; // For Every Frame

        if( frame.empty() ) // No frames anymore, done
        {
            std::cout<<"No frames anymore, break the loop..."<<std::endl;
            break;
        }

	 frame.copyTo( backupFrame );
	
	 float dW = g_projectData->mBarrelDistortion;
	 float dWinv = 1.0 / g_projectData->mBarrelDistortion;
	 float d2Tan = 2.0 * tan( dW / 2.0 );
	
	 cv::Mat udImage;
	 
     // Undistort the image
	 undistortImageBarrel( frame , udImage , dW , dWinv , d2Tan , cameraMatrix.at< double >( 0 , 0 ) ,
			       cameraMatrix.at< double >( 1 , 1 ) , cameraMatrix.at< double >( 0 ,2 ) , 
			       cameraMatrix.at< double >( 1 , 2 ) );

	g_currentImage = udImage; // undistorted image
	
	ptam( backupFrame ); // Use PTAM to get the pose
	
	g_ptamDrawImage->setImage( backupFrame );
	
	// computeStereoCorresp();
	
	if( !g_runDTAM ) // Is DTAM running?
	 continue;

	udImage.convertTo( image , CV_32FC3 ,1.0/255.0);

	Eigen::Matrix4f pose2 = g_currentPose; // pose.transpose() * initialPose;
	
    T.at< double >( 0 , 0 ) = pose2( 0 , 3 ); // =Ts[imageNum];
    T.at< double >( 1 , 0 ) = pose2( 1 , 3 );
	T.at< double >( 2 , 0 ) = pose2( 2 , 3 );
	
	for( int rr = 0; rr < 3; rr++ )
	  for( int cc = 0; cc < 3 ; cc++ )
	  {
	    R.at< double >( rr , cc ) = pose2( rr , cc );
	  }
	  
	if( g_NewKeyFrame ) // Got new keyframe, need to init a new Cost Volume
	{
	  std::cout<<" Init cost volume "<<cameraMatrix<<std::endl;
	  
	  float near = 1.0 / g_minDistance ;
	  float far = 1.0 / g_maxDistance ;
	  
	  cv = CostVolume( image, (FrameID)0, layers, near, far, R, T, cameraMatrix);
	  
	  computeRays( udImage.cols , udImage.rows , g_currentPose.block( 0 , 0 , 3 , 3 ) , g_currentPose.block( 0 , 3 , 3 , 1 ) , g_intrinsics , rays );
	  
	  cameraCenter = -g_currentPose.block( 0 , 0 , 3 , 3 ).transpose() * g_currentPose.block( 0 , 3 , 3 , 1 );
	  imageNum++;
	  
	  g_NewKeyFrame = false;
	 
	  continue;
	}
	
	if( imageNum == 0 )
	  continue;
	
	 imageNum++;
	
        if(cv.count < imagesPerCV){
            cv.updateCost(image, R, T);
        }
        else{
            // Attach optimizer
            Ptr<DepthmapDenoiseWeightedHuber> dp = createDepthmapDenoiseWeightedHuber(cv.baseImageGray, cv.cvStream);
            DepthmapDenoiseWeightedHuber& denoiser = *dp;
            Optimizer optimizer(cv);
            optimizer.initOptimization();
            GpuMat a(cv.loInd.size(), cv.loInd.type());
            cv.cvStream.enqueueCopy(cv.loInd, a);
            GpuMat d;
            denoiser.cacheGValues();
            ret = image*0;

            pfShow("A function", ret, 0, cv::Vec2d(0, layers));
            pfShow("D function", ret, 0, cv::Vec2d(0, layers));
            pfShow("A function loose", ret, 0, cv::Vec2d(0, layers));
            pfShow("Predicted Image", ret, 0, Vec2d(0,1));
            pfShow("Actual Image", udImage);
            
            std::cout<<" start optimization "<<std::endl;

            bool doneOptimizing; int Acount=0; int QDcount=0;
            do{
               a.download(ret);
               pfShow("A function", ret, 0, cv::Vec2d(0, layers));

                for (int i = 0; i < 10; i++) {
                   d = denoiser(a, optimizer.epsilon, optimizer.getTheta());
                   QDcount++;
                    
                   d.download(ret);
                   pfShow("D function", ret, 0, cv::Vec2d(0, layers));
                }
                doneOptimizing = optimizer.optimizeA(d, a);
                Acount++;
            }while(!doneOptimizing);

            optimizer.lambda = .01;
            optimizer.optimizeA(d, a);
            optimizer.cvStream.waitForCompletion();
            a.download(ret);
            pfShow("A function loose", ret, 0, cv::Vec2d(0, layers));

            imwrite("outz.png",ret);

            // start over from image number 0 for a new cost volume
            imageNum=0;
	    
	    std::cout<<" done optimization "<<std::endl;
	    
	    Mat out =optimizer.depthMap();
        computeAndDisplayPoints( rays , out , udImage , cameraCenter , cv );
            
        a.download(ret);
            
        }
        s.waitForCompletion(); // so we don't lock the whole system up forever
    }
    std::cout<<"Waiting for GPU Async Tasks to complete..."<<std::endl;
    s.waitForCompletion();
    Stream::Null().waitForCompletion();
    return 0;
}

void init3dViewer()
{
  g_displayModes = PTAM_IMAGE;
  
  bool fullscreen = false;
  
  static const char windowTitle[] = "OpenDTAM glViewer";
  
  if (not glfwInit()) {
    printf("Failed to initialize GLFW\n");
    return;
  }
  
  setGLCoreProfile();
  
  if (fullscreen) {

    g_primary = glfwGetPrimaryMonitor();

    if (not g_primary) {
        int count=0;
        GLFWmonitor ** monitors = glfwGetMonitors(&count);

        if (count)
            g_primary = monitors[0];
    }

    if (g_primary) {
        GLFWvidmode const * vidmode = glfwGetVideoMode(g_primary);
        g_width = vidmode->width;
        g_height = vidmode->height;
        }
    }

    if (not (g_window = glfwCreateWindow(g_width, g_height, windowTitle,
                                       fullscreen and g_primary ? g_primary : NULL, NULL))) {
        printf("Failed to open window.\n");
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(g_window); // Set the OpenGL context

    glfwGetFramebufferSize(g_window, &g_width, &g_height);
    glfwSetFramebufferSizeCallback(g_window, reshape);

    // Event Callback Handlers
    glfwSetKeyCallback(g_window, keyboard);
    glfwSetCursorPosCallback(g_window, motion);
    glfwSetMouseButtonCallback(g_window, mouse);
    glfwSetScrollCallback( g_window , scroll );
    glfwSetWindowCloseCallback(g_window, windowClose);

    // This is the only way to initialize glew correctly under core profile context.
    glewExperimental = true;
    
    if ( GLenum r = glewInit() != GLEW_OK) 
    {
        printf("Failed to initialize glew. Error = %s\n", glewGetErrorString(r));
        exit(1);
    }
    
    GLenum	err = glGetError();
    
    if (err != GL_NO_ERROR)
    {
      qDebug() << "Shader OpenGL error : "  << ( char * )gluErrorString( err ) <<endl;
    }
    
    // Cameras
    g_camera = new TrackBallCamera();
    g_camera->setViewPortDimension( g_width, g_height );
    
    g_ptamCamera = new TrackBallCamera();
    g_ptamCamera->setViewPortDimension( g_width , g_height );

    g_OpenCVStereoDenseCamera = new TrackBallCamera();
    g_OpenCVStereoDenseCamera->setViewPortDimension( g_width , g_height );
    
    // For renderer
    g_mesh = new vc::DenseData();
    
    g_ptamSparseData = new vc::DenseData();
    
    g_DTAMDenseData = new vc::DenseData();
    
    g_OpenCVStereoDenseData = new vc::DenseData();
    
    g_ptamDrawImage = new vc::DrawImage();
    
    g_disparityImage = new vc::DrawImage();
    
    g_mesh->generateVertexArrays();
    g_mesh->init();
    g_mesh->setCamera( g_camera );
    
    g_ptamSparseData->generateVertexArrays();
    g_ptamSparseData->init();
    g_ptamSparseData->setCamera( g_ptamCamera  );
    
    g_DTAMDenseData->generateVertexArrays();
    g_DTAMDenseData->init();
    g_DTAMDenseData->setCamera( g_ptamCamera );
    
    g_OpenCVStereoDenseData->generateVertexArrays();
    g_OpenCVStereoDenseData->init();
    g_OpenCVStereoDenseData->setCamera( g_OpenCVStereoDenseCamera );
    
    g_ptamDrawImage->generateVertexArray();
    g_ptamDrawImage->init();
    
    g_disparityImage->generateVertexArray();
    g_disparityImage->init();
    
    // Start the main drawing loop
    while (g_running) {
        // idle();
        display();
        
        glfwPollEvents();
        glfwSwapBuffers(g_window);

        glFinish();
    }
}


// Camera intrinsic 
void setIntrinsicsPTAMType( float fx , float fy , float cx , float cy , float barrelDist )
{
  g_ptamCamParams( 0 ) = fx ;
  g_ptamCamParams( 1 ) = fy ;
  g_ptamCamParams( 2 ) = cx ;
  g_ptamCamParams( 3 ) = cy ;
  g_ptamCamParams( 4 ) = barrelDist ;
  
  g_intrinsics.setIdentity();
  
  g_intrinsics( 0 , 0 ) = fx * 640;
  g_intrinsics( 1 , 1 ) = fy * 480;
  g_intrinsics( 0 , 2 ) = cx * 640;
  g_intrinsics( 1 , 2 ) = cy * 480;
  
}

void computeRays( int w , int h , const Eigen::Matrix3f  &rotation , const Eigen::Vector3f &translation , Eigen::Matrix3f &K , cv::Mat &rays   )
{
	rays.create( h , w , CV_32FC3 );

	Eigen::Matrix3f krInv = ( K * rotation ).inverse();

	for( int xx = 0; xx < w ; xx++ )
		for( int yy = 0; yy < h ; yy++ )
		{
			Eigen::Vector3f vec( xx , yy , 1 );

			Eigen::Vector3f ray = krInv * vec;

			ray.normalize();

			rays.at< Eigen::Vector3f >(  yy , xx ) = ray;
		}
}

void computeAndDisplayPoints( Mat& rays, Mat& depths, cv::Mat &colorFrame , Eigen::Vector3f& cameraCenter, CostVolume& cost )
{
	int w = rays.cols;
	int h = rays.rows;

	std::vector< Eigen::Vector3f > objectPoints, colors;

	objectPoints.reserve( w * h );

	float maxDepth = cost.near;
	float minDepth = cost.far;
	
	float step = ( maxDepth - minDepth ) / DTAM_LAYERS;
	
	minDepth = maxDepth - ( DTAM_LAYERS - 1 ) * step;

    Mat base;
    cost.baseImage.download(base);
    float* colorData = (float*)base.data;

	int x , y;

		for( int yy = 0; yy < (float)h ; yy++ )
		  	for( int xx = 0; xx < (float)w ; xx++ )
		{
		  
		  
			if( depths.at< float >( yy , xx ) < minDepth || depths.at< float >( yy , xx ) > maxDepth )
			{
			    colorData += 3;
				continue;
			}
			
			
			Eigen::Vector3f vec = cameraCenter + ( 1.0 / depths.at< float >( yy , xx ) ) * rays.at< Eigen::Vector3f >( yy , xx );
  	        objectPoints.push_back( vec );
			
			colors.push_back( Eigen::Vector3f( colorData[ 2 ], colorData[ 1 ], colorData[ 0 ]) );
			colorData += 3;
		}
		
		g_DTAMDenseData->setCameraToInitialized();
				
		g_DTAMDenseData->setData( objectPoints , colors );
}


void displayImage( cv::Mat &image )
{
  g_ptamDrawImage->setImage( image );
  
}

void update()
{

}


void display()
{
  if( g_displayModes == PTAM_IMAGE )
  {
    if( g_ptamDrawImage )
    {
      GL_CHECK( glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) );
      g_ptamDrawImage->setViewPort( g_width , g_height );
      g_ptamDrawImage->render();
    }
    
  }
  else if( g_displayModes == PTAM_SPARSE )
  {
    if( g_ptamSparseData )
    {
      //qDebug() << " rendering sparse data "<<endl;
      GL_CHECK( glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) );
      g_ptamCamera->setViewPortDimension( g_width , g_height );
      g_ptamSparseData->render();
    }
    
  }
  else if( g_displayModes == DTAM_DENSE )
  {
    if( g_DTAMDenseData )
    {
      //qDebug() << " rendering data "<<endl;
      GL_CHECK( glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) );
      g_ptamCamera->setViewPortDimension( g_width , g_height );
      g_DTAMDenseData->render();
    }
  }
  else if( g_displayModes == OPENCV_STEREO_DENSE )
  {
    if( g_OpenCVStereoDenseData )
    {
      //qDebug() << " rendering data "<<endl;
      GL_CHECK( glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) );
      g_OpenCVStereoDenseCamera->setViewPortDimension( g_width , g_height );
      g_OpenCVStereoDenseData->render();
    }
    
    
  }
  else if( g_displayModes == STEREO_DISPARITY )
  {
    if( g_disparityImage )
    {
        GL_CHECK( glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) );
        g_disparityImage->setViewPort( g_width , g_height );
        g_disparityImage->render();
    }
    
  }
}


void scroll(GLFWwindow* window,double x,double y){
  int zoom = y * 250;
  
  g_camera->registerMouseWheel( zoom );
  
  if( g_displayModes == PTAM_SPARSE )
   g_ptamCamera->registerMouseWheel( zoom );
  
  if( g_displayModes == DTAM_DENSE )
   g_ptamCamera->registerMouseWheel( zoom );
  
  if( g_displayModes == OPENCV_STEREO_DENSE )
  g_OpenCVStereoDenseCamera->registerMouseWheel( zoom );
  
}


static void
#if GLFW_VERSION_MAJOR >= 3
motion(GLFWwindow *, double dx, double dy) {
    int x = (int)dx, y = (int)dy;
#else
motion(int x, int y) {
#endif
  
    QPoint pos( x , y );
    
    Qt::MouseButtons bt;

    if ( g_mbutton[0] && !g_mbutton[1] && !g_mbutton[2] ) 
    {
        // orbit
        g_camera->registerMouseMove( pos , Qt::LeftButton );
	
	if( g_displayModes == PTAM_SPARSE || g_displayModes == DTAM_DENSE )
	  g_ptamCamera->registerMouseMove( pos , Qt::LeftButton );
	
	if( g_displayModes == OPENCV_STEREO_DENSE )
        g_OpenCVStereoDenseCamera->registerMouseMove( pos , Qt::LeftButton );
	
    } 
    else if ( !g_mbutton[0] && g_mbutton[1] && !g_mbutton[2]) 
    {
        g_camera->registerMouseMove( pos , Qt::RightButton );
	
	if( g_displayModes == PTAM_SPARSE || g_displayModes == DTAM_DENSE )
	g_ptamCamera->registerMouseMove( pos , Qt::RightButton );
	
	if( g_displayModes == OPENCV_STEREO_DENSE )
        g_OpenCVStereoDenseCamera->registerMouseMove( pos , Qt::RightButton );
    } 
    
    g_prev_x = x;
    g_prev_y = y;
}

static void mouse(GLFWwindow *, int button, int state, int mods) 
{

    if ( button < 3 ) 
    {
        g_mbutton[ button ] = (state == GLFW_PRESS);
	
	    QPoint pos( g_prev_x , g_prev_y );
	
	if( g_mbutton[ button ] )
	{
	  if( button == 0 )
	   g_camera->registerMousePress( pos , Qt::LeftButton );
	  
	  if( g_displayModes == PTAM_SPARSE || g_displayModes == DTAM_DENSE )
	   g_ptamCamera->registerMousePress( pos , Qt::LeftButton );
	  
	  if( g_displayModes == OPENCV_STEREO_DENSE )
           g_OpenCVStereoDenseCamera->registerMousePress( pos , Qt::LeftButton );
	  
	  if( button == 1 )
	  {
            g_camera->registerMousePress( pos , Qt::RightButton );
	    
	    if( g_displayModes == PTAM_SPARSE || g_displayModes == DTAM_DENSE )
	    g_ptamCamera->registerMousePress( pos , Qt::RightButton );
	    
	    if( g_displayModes == OPENCV_STEREO_DENSE )
            g_OpenCVStereoDenseCamera->registerMousePress( pos , Qt::RightButton );
	  }
	  
	}
	else if( state == GLFW_RELEASE )
	{
	  g_camera->registerMouseRelease( pos );
	  
	  if( g_displayModes == PTAM_SPARSE || g_displayModes == DTAM_DENSE )
	  g_ptamCamera->registerMouseRelease( pos );
	  
	  if( g_displayModes == OPENCV_STEREO_DENSE )
          g_OpenCVStereoDenseCamera->registerMouseRelease( pos );
	}
	
    }
}

static void
keyboard(GLFWwindow *, int key, int /* scancode */, int event, int /* mods */) {


    switch (key) {
        case 'Q': g_running = 0; break;
        case ' ': g_spaceBarPressed++; break;
        case '=': g_mesh->resetCamera();std::cout<<" Reset camera "<<std::endl; break;
        case 'D': g_runDTAM = true; std::cout<<" Run dtam "<<std::endl; break;
        case 'S': g_displayModes = PTAM_SPARSE; std::cout<<" Render sparse points "<<std::endl; break;
        case 'I': g_displayModes = PTAM_IMAGE;  std::cout<<" Render Image "<<std::endl; break;
        case 'P': g_displayModes = DTAM_DENSE;  std::cout<<" Render dtam dense points "<<std::endl; break;
        case 'O': g_displayModes = OPENCV_STEREO_DENSE; std::cout<<" Render opencv stereo dense points "<<std::endl; break;
        case 'G': g_displayModes=STEREO_DISPARITY; std::cout<<" Render opencv stereo dense points "<<std::endl; break;
    }
}

void ptam( Mat& frame  )
{
  runPtam( frame , g_projectData );
}

void grabFrame( cv::Mat& image , CVD::Image<CVD::Rgb<CVD::byte> > &coloredImage , CVD::Image< CVD::byte > &grayImage  )
{
    int w = image.cols;
    int h = image.rows;
  
    int size = w * h;
    
    CVD::ImageRef sizeG = grayImage.size();
    CVD::ImageRef sizeC = coloredImage.size();
    
    CVD::ImageRef refSize( w , h );
    
    if( sizeG.x != w || sizeG.y != h )
    {
      grayImage.resize( refSize );
    }
    
    if( sizeC.x != w || sizeC.y != h )
    {
      coloredImage.resize( refSize );
    }
    
    uchar *g = grayImage.data();
    CVD::Rgb< CVD::byte > *c = coloredImage.data();
  
    uchar *data = image.data;
    
    cv::Mat gImage;
    
    cv::cvtColor( image , gImage , cv::COLOR_RGB2GRAY );
    
    uchar *grayImageData = gImage.data;
  
    for( int ss = 0; ss < size ; ss++ ) 
    {
      g[ ss ] = grayImageData[ ss ];
      
      c[ ss ].blue = data[ 3 * ss ];
      c[ ss ].green = data[ 3 * ss + 1 ] ;
      c[ ss ].red = data[ 3 * ss + 2 ] ;
    }  
}


void runPtam( cv::Mat& image , ProjectData *projectData )
{
  
  cv::Mat backupImage = image;

  Eigen::Matrix3f K;

  K.setIdentity();

  if( !projectData->mpTracker )
  {
    int w = image.cols;
    int h = image.rows;
    
    CVD::ImageRef refSize( w , h );
    
    TooN::Vector< 5 > camParams;

    camParams[ 0 ] = g_ptamCamParams( 0 );
    camParams[ 1 ] = g_ptamCamParams( 1 );
    camParams[ 2 ] = g_ptamCamParams( 2 );
    camParams[ 3 ] = g_ptamCamParams( 3 );
    camParams[ 4 ] = g_ptamCamParams( 4 );
    
    g_intrinsics.setIdentity();
    
    g_intrinsics( 0 , 0 ) = g_ptamCamParams( 0 ) * image.cols;
    g_intrinsics( 1 , 1 ) = g_ptamCamParams( 1 ) * image.rows;
    g_intrinsics( 0 , 2 ) = g_ptamCamParams( 2 ) * image.cols - 0.5;
    g_intrinsics( 1 , 2 ) = g_ptamCamParams( 3 ) * image.rows - 0.5;
    
    g_projectData->mBarrelDistortion = g_ptamCamParams( 4 );
    
    projectData->mpCamera = new ATANCamera( camParams , w , h );
    
    projectData->mpMap = new Map();
    
    projectData->mpMapMaker = new MapMaker( *(projectData->mpMap) , *(projectData->mpCamera) );    
    
    projectData->mpTracker = new Tracker( refSize , *( projectData->mpCamera ) , 
					                      *( projectData->mpMap ) , 
					                      *( projectData->mpMapMaker ) );    

  }
  
  grabFrame( image , projectData->mimFrameRGB , projectData->mimFrameBW );
  
  QString displayMessage;
  
  std::vector< QPoint > fastCorners , trackedCorners ;
  std::vector< std::pair< QPoint , QPoint > > trailPoints;
  std::vector< QVector3D > objectPoints;

  QMatrix4x4 pose;
  
  // Track the frame using PTAM
  projectData->mpTracker->TrackFrameCustom( 
          projectData->mimFrameBW , 
          projectData->mPtamData.mMessage ,  
          projectData->mPtamData.mSpaceBarPressed,
          fastCorners, trackedCorners, 
          objectPoints, trailPoints, 
          pose );
  
  memcpy( g_currentPose.data() , pose.data() , 16 * sizeof( float )  );

  if( projectData->mRenderMessage )
  {
      projectData->mRenderMessage = false;

      // projectData->renderPTAMMessage();
  }

  // Draw the trails if necessary
  if( trailPoints.size() > 0 )
  {
    drawTrailPoints( trailPoints , image );
  }

  // Draw the tracked corner points
  if( trackedCorners.size() > 0 )
  {
      drawTrackedPoints( trackedCorners , image );
  }
  
  projectData->mPtamData.mNumFrames++;
  
  projectData->mPtamData.mFrameLatency++;
  
  if( projectData->mPtamData.mFrameLatency >= projectData->mPtamData.mNumFrameToSkip )
  {
    projectData->mPtamData.mFrameLatency = 0;
    // projectData->mPtamData.mSkipFrame = false;
  }
  else
  {
    // projectData->mPtamData.mSkipFrame = true;
  }

  // Draw 3d points after adding new KF
  if( projectData->mpMap->vpKeyFrames.size() > projectData->mPtamData.mPrevNumCams  )
  {
      
      g_NewKeyFrame = true; // Add new keyframe
      
      g_computeDepthMap++; // New depth map added to the model
      
      g_projectData->mPrevKeyImage = g_projectData->mCurrentKeyImage;
      
      g_projectData->mCurrentKeyImage = g_currentImage;
      
      if( projectData->mpMap->vpKeyFrames.size() == 2 )
      {
	     projectData->mPtamData.mNumFrames = 0;
	
	     projectData->mPtamData.mSaveFrameIndex = 0;
	
	     projectData->mPtamData.mPtamInitialized = true;
      }
          
      projectData->mPtamData.mPrevNumCams = projectData->mpMap->vpKeyFrames.size(); // Save old #KF

      std::cout<<" Number of PTAM points and Keyframes : "<< projectData->mpMap->vpPoints.size() <<" "<<projectData->mpMap->vpKeyFrames.size()<<std::endl;
     
      int numPoints = projectData->mpMap->vpPoints.size();
      
      std::vector< Eigen::Vector3f > points( numPoints ) , filteredPoints , colors( numPoints );

      double pt[ 3 ];
      
      // Number of object points [w*h]
        if( objectPoints.size() > 200 )
  {

    Eigen::Matrix< float , 3 , 4 > projMat;

    for( int rr = 0; rr < 3; rr++ )
	  for( int cc = 0; cc < 4 ; cc++ )
	  {
		  projMat( rr , cc ) = pose( rr , cc );
	  }

    // std::cout<<projMat<<std::endl;

    projMat = g_intrinsics * projMat;

	//  std::cout<<K<<std::endl;

    // Object points in homogeous
    Eigen::Vector4f objPtHmg( objectPoints[ 200 ][ 0 ] , objectPoints[ 200 ][ 1 ] , objectPoints[ 200 ][ 2 ] , 1.0 );

    Eigen::Vector3f projHmg = projMat * objPtHmg;

    // Point on image in Pixel unit
    float xp = ( projHmg( 0 ) / projHmg( 2 ) );
    float yp = ( projHmg( 1 ) / projHmg( 2 ) );

  }

      for( int pp = 0; pp < numPoints; pp++ )
      {
          TooN::Vector< 3 > v3Pos = projectData->mpMap->vpPoints[pp]->v3WorldPos; // World position

          points[ pp ]( 0 ) = v3Pos[ 0 ] ;
          points[ pp ]( 1 ) = v3Pos[ 1 ] ;
          points[ pp ]( 2 ) = v3Pos[ 2 ] ;
	  
          colors[ pp ]( 0 ) = 0;
          colors[ pp ]( 1 ) = 0.5;
          colors[ pp ]( 2 ) = 0;
      }

    // Camera Center
    Eigen::Vector3f cameraCenter = -g_currentPose.block( 0 , 0 , 3 , 3 ).transpose() * g_currentPose.block( 0 , 3 , 3 , 1 );
      
    // Filter points according to distance, points->filteredPoints
    filterPoints( cameraCenter , points , filteredPoints , g_minDistance , g_maxDistance );
    
    // Set the renderer data of PTAM
    g_ptamSparseData->setData( filteredPoints , colors );
      
    if( numPoints > 0 )
    {
      if( !g_ptamInitialized )
      {
        g_ptamInitialized = true;
        g_ptamSparseData->resetCamera();

        std::cout<<"PTAM init complete, ready to run DTAM!"<<std::endl;

        // g_runDTAM = true;
      }
    }
 
  }
  
    if( g_spaceBarPressed == 2 )
  {
    projectData->mPtamData.mSpaceBarPressed = true;
    
    g_spaceBarPressed = 0;
    
    std::cout<<" Space bar pressed "<<std::endl;
  }
}

/**
 * Draw trail points(line) on the image
 */
void drawTrailPoints( std::vector< std::pair< QPoint, QPoint> > &points , cv::Mat &image )
{
  int numTrailPoints = points.size();

  for( int pp = 0; pp < numTrailPoints; pp++ )
  {
      cv::Point pt1 , pt2;
      
      pt1.x = points[ pp ].first.x();
      
      pt1.y = points[ pp ].first.y();
      
      pt2.x = points[ pp ].second.x();
      
      pt2.y = points[ pp ].second.y();
    
      cv::line( image , pt1 , pt2 , cv::Scalar(  255 , 0 , 0 ) );
  }
}


/**
 * Draw fast corners on the image
 */
void rawFastCorners( std::vector< QPoint > &points , cv::Mat &image )
{
   int numPoints = points.size();

   for( int pp = 0; pp < numPoints ; pp++ )
   {
     
     cv::Point pt ;
     
      pt.x = points[ pp ].x();
      
      pt.y = points[ pp ].y();
      
      cv::circle(image , pt , 2 , cv::Scalar(0 , 255 , 0  ) );
   }
}

/**
 * Draw tracked points on the image
 */
void drawTrackedPoints( vector< QPoint >& trackedPoints, Mat& image )
{
    int numPoints = trackedPoints.size();

    for( int pp = 0; pp < numPoints ; pp++ )
    {
             cv::Point pt ;
     
      pt.x = trackedPoints[ pp ].x();
      
      pt.y = trackedPoints[ pp ].y();
      
      cv::circle(image , pt , 2 , cv::Scalar( 0 , 0 , 255 ) );
    }
}


/*
 * Compute color at [x,y] using Bilinear interpolation
 * The result stores to color
 */
void imageValColored( float x , float y  , uchar *image , int stride , uchar color[ 3 ]  )
{
    // Compute color using bilinear interpolation
	float valR = 0 , valG = 0 , valB = 0;

	// Compute bilinear interpolation
	const int lx = (int)floor( x );
	const int ly = (int)floor( y );

	int _x = (int)x;
	int _y = (int)y;

	const float dx1 = x - lx;  
	const float dx0 = 1.0f - dx1;
	const float dy1 = y - ly;  
	const float dy0 = 1.0f - dy1;

	const float f00 = dx0 * dy0;  
	const float f01 = dx0 * dy1;
	const float f10 = dx1 * dy0;  
	const float f11 = dx1 * dy1;

	uchar* ucp0B = image + _y * stride + 3 * _x ;
	uchar* ucp0G = image + _y * stride + 3 * _x + 1 ;
	uchar* ucp0R = image + _y * stride + 3 * _x + 2 ;
	
	uchar* ucp1B = image  + ( _y + 1 )  * stride + 3 * _x ;
	uchar* ucp1G = image  + ( _y + 1 )  * stride + 3 * _x + 1 ;
	uchar* ucp1R = image  + ( _y + 1 )  * stride + 3 * _x + 2 ;

	valR += *( ucp0R ) * f00 + *( ucp1R ) * f01;  
	valR += *( ucp0R + 3 ) * f10 + *( ucp1R + 3 ) * f11; 

	valG += *( ucp0G ) * f00 + *( ucp1G ) * f01;  
	valG += *( ucp0G + 3 ) * f10 + *( ucp1G + 3 ) * f11;

	valB += *( ucp0B ) * f00 + *( ucp1B ) * f01;  
	valB += *( ucp0B + 3 ) * f10 + *( ucp1B + 3 ) * f11;

	color[ 0 ] = ( uchar )valB;
	color[ 1 ] = ( uchar )valG;
	color[ 2 ] = ( uchar )valR;
}


void undistortImageBarrel( cv::Mat &image , cv::Mat &udImage , double dw , double dwInv , 
   	            double d2Tan , double fx , double fy , double cx , double cy )
{
   udImage.create( image.size() , image.type() );

   udImage.setTo( cv::Scalar( 0 ) );

   int w = image.cols;
   int h = image.rows;

   double fxInv = 1.0 / fx;
   double fyInv = 1.0 / fy;

   uchar *imageData = image.data;
   uchar *uImageData = udImage.data;

   for( int x  = 0; x < w ; x++ )
   	for( int y = 0; y < h ; y++ )
   	{
   		double ux = ( x - cx ) * fxInv;
   		double uy = ( y - cy ) * fyInv;

   		double r = sqrt( ux * ux + uy * uy );

   		double scale = dwInv * atan(r * d2Tan) / r;

   		ux *= scale;
   		uy *= scale;

   		float xud = ux * fx + cx;
   		float yud = uy * fy + cy;

   		if( xud > 0 && xud < w - 2 && yud > 0 && yud < h - 2 )
   		{
   			imageValColored( xud , yud , imageData , 3 * w , ( uImageData + 3 * y * w + 3 * x )  );
   		}

   	}
}
 
bool distanceSortPredicate( const std::pair< double , int > &obj1 , const std::pair< double , int > &obj2 )
{
  return obj1.first < obj2.first;
}
 
 
void filterPoints( Eigen::Vector3f &cameraCenter , std::vector< Eigen::Vector3f > &points , std::vector< Eigen::Vector3f > &filteredPoints, float &minDistance , float &maxDistance )
{
    int numPoints = points.size();
    
    std::pair< double , int > distancePointPair;
    
    std::vector< std::pair< double , int > > distances( numPoints );
    
    for( int pp = 0; pp < numPoints ; pp++ )
    {
      distances[ pp ].first = ( points[ pp ] - cameraCenter ).norm();
      distances[ pp ].second = pp;
    }
    
    std::sort( distances.begin() , distances.end() , distanceSortPredicate );
    
    int numAvgPts = 0.8 * numPoints;
    
    double avgDist = 0;
    
    for( int aa = 0; aa < numAvgPts ; aa++ )
    {
        avgDist += distances[ aa ].first;
    }
    
    avgDist /= numAvgPts;
    
    double filterDistance = 6 * avgDist;
    
    filteredPoints.reserve( numPoints );
    
    for( int pp = 0; pp < numPoints ; pp++ )
    {
      double distance = (points[ pp ] - cameraCenter ).norm();
      
      if( distance < filterDistance )
      {
        filteredPoints.push_back( points[ pp ] );  
      }
    }
    
    maxDistance = filterDistance;
    minDistance = distances[ 0 ].first ;//* 0.5;
    
 	// std::cout<<" min max distances : "<<minDistance<<" "<<maxDistance<<std::endl; 
    
}

void computeStereoCorresp()
{
  int numKeyFrames = g_projectData->mpMap->vpKeyFrames.size();
  
  if( numKeyFrames < 5 || !g_computeDepthMap  )
    return;
  
  KeyFrame *kf1 = g_projectData->mpMap->vpKeyFrames[ numKeyFrames - 2 ];
  KeyFrame *kf2 = g_projectData->mpMap->vpKeyFrames[ numKeyFrames - 1 ];
  
  TooN::Matrix< 3 , 3 > tRot1 = kf1->se3CfromW.get_rotation().get_matrix();
  TooN::Vector< 3 > tTrans1 = kf1->se3CfromW.get_translation();
  
  TooN::Matrix< 3 , 3 > tRot2 = kf2->se3CfromW.get_rotation().get_matrix();
  TooN::Vector< 3 > tTrans2 = kf2->se3CfromW.get_translation();
  
  Eigen::Matrix4f pose1 , pose2 , pose;
  
  pose1.setIdentity();
  pose2.setIdentity();
  
  for( int rr = 0; rr < 3; rr++ )
  {
    for( int cc = 0 ; cc < 3 ; cc++ )
    {
      pose1( rr , cc ) = tRot1( rr , cc );
      pose2( rr , cc ) = tRot2( rr , cc );
    }
    
    pose1( rr , 3 ) = tTrans1[ rr ]; 
    pose2( rr , 3 ) = tTrans2[ rr ];
  }
  
  pose = pose2 * pose1.inverse();
  
  cv::Mat  R( 3 , 3 , CV_64FC1 ) , T( 3 , 1 , CV_64FC1 ) , K( 3 , 3 , CV_64FC1 ) , D( 5 , 1 , CV_64FC1 ) ;
  
  D.setTo( cv::Scalar( 0 ) );
  
  for( int rr = 0; rr < 3; rr++ )
  {
    for( int cc = 0 ; cc < 3 ; cc++ )
    {
      R.at< double >( rr , cc ) = pose( rr , cc );
      
      K.at< double >( rr , cc ) = g_intrinsics( rr , cc );
    }
    
    T.at< double >( rr , 0 ) = pose( rr , 3 ); 
  }
  
  cv::Mat reconstruction, color ;
  
  tr::DepthMapStereo stereo;
  
  stereo.setDrawImage( g_disparityImage );
  
  std::vector< Eigen::Vector3f > reconsPts , reconsColors;
  
  stereo.computeStereoCorrespondence( g_projectData->mPrevKeyImage , g_projectData->mCurrentKeyImage , reconstruction, color , R , T , K , D  , 0 ,reconsPts , reconsColors , -1  );

  g_OpenCVStereoDenseData->setData( reconsPts , reconsColors );
  
  g_OpenCVStereoDenseData->resetCamera();
  
  g_computeDepthMap = 0;
}

// 3D Viewer
void ViewerThread::run()
{
  init3dViewer();
}

// Main thread running DTAM mapper
void MainThread::run()
{
  App_main( argc , argv );
}


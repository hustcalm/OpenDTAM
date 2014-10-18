#include "opencv/highgui.h"
#include "external_includes/opencvincludes.h"
#include "VideoSource.h"
#include <gvars3/instances.h>

using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace cv;

VideoSource::VideoSource()
{
  cout << " VideoSource_Linux: Opening video source...";

  VideoCapture* cap = new VideoCapture();

//   cap->set(CV_CAP_PROP_FRAME_WIDTH, 640);
//   cap->set(CV_CAP_PROP_FRAME_HEIGHT, 480);

 

  cap->open("/home/avanindra/Downloads/DSC_0008.MOV");//( 0 );
  
  //cv::String fileName( "D:\\projects\\project_data\\video_datasets\\DSCF0061.avi" );
  
  //cap->open( fileName );

 // cap->open//( "/media/avanindra/Data/projects/project_data/video_datasets/chessboard_video_1920x1080.AVI");

  if( !cap->isOpened() )
  {
    cerr << "Unable to get the camera" << endl;
    exit(-1);
  }

 cout << " ... got video source." << endl; 
 mirSize = ImageRef(640,484);

 mptr = cap;  
};

ImageRef VideoSource::Size()
{
return mirSize;
};

void conversionNB(Mat &frame, Image<byte> &imBW)
{
 
	
uchar* frame_p = frame.data;
 
 for (int i = 0; i < frame.rows; i++)
 {
   for (int j = 0; j < frame.cols; j++)
   { 
     imBW[i][j] = ( ( int )frame_p[ 0 ] + ( int )frame_p[ 1 ] + ( int )frame_p[2] ) / 3;

	 frame_p += 3;
   }
}

}

void conversionRGB(Mat &frame, Image< CVD::Rgb<CVD::byte> > &imRGB){
uchar* frame_p = frame.data;
for (int i = 0; i < frame.rows; i++){
for (int j = 0; j < frame.cols; j++){
imRGB[i][j].red = frame_p[0];
imRGB[i][j].green = frame_p[1];
imRGB[i][j].blue = frame_p[2];

frame_p += 3;
}
}
}

void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
{
Mat frame;
VideoCapture* cap = (VideoCapture*)mptr;
*cap >> frame;

// std::cout<<frame.cols<<" "<<frame.rows<<std::endl;

// cv::resize( frame , frame , cv::Size( 640 , 480 ) );

conversionNB(frame, imBW);
conversionRGB(frame, imRGB);
frame.release();
}
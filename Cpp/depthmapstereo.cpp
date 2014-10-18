/*
    Copyright (c) 2011, <copyright holder> <email>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY <copyright holder> <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL <copyright holder> <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "depthmapstereo.h"
#include "external_includes/stlincludes.h"
#include "iostream"

using namespace cv;


namespace tr
{


DepthMapStereo::DepthMapStereo()
{
 bm.state->preFilterCap = 31;
 bm.state->minDisparity = 10;

 bm.state->textureThreshold = 10;
 bm.state->uniquenessRatio = 15;
 bm.state->speckleWindowSize = 100;
 bm.state->speckleRange = 32;
 bm.state->disp12MaxDiff = 1;
   
 sgbm.preFilterCap = 63;
 sgbm.SADWindowSize = 7;  

 sgbm.minDisparity = 10;
 sgbm.uniquenessRatio = 10;
 sgbm.speckleWindowSize = 100;
 sgbm.speckleRange = 32;
 sgbm.disp12MaxDiff = 0;
 sgbm.fullDP = false;
}
  



/*This method transposes an image. It's useful in case of vertical paralax stereo.*/

void DepthMapStereo::transposeImage(Mat& src, Mat& dst)
{
   
 dst = Mat(src.cols,src.rows,src.type());
 
 int channels = src.channels();
 
 if(channels == 1)
 {
   transpose(src,dst);
   return;
 }
 
 uchar *ptr1 = (uchar *)src.data;
 uchar *ptr2 = (uchar *)dst.data;
 
 for(int y = 0; y < dst.rows; y++)
    {
        for(int x = 0; x < dst.cols; x++)
        {
	  ptr1 = (uchar *)src.data + x * src.step;
	  ptr2 = (uchar *)dst.data + y * dst.step;
	  
	  for(int i = 0; i < channels; i++)
	  {
	    ptr2[channels * x + i ] = ptr1[ channels * y + i ];	   
	  }
	}
    }

}




/*
 *TODO: 
 * 1)clean, stablize and document this method.
 * 2)write a similar method using IVT library and retain the reconstruction in
 *   global co-ordinate system.
 * 
 * cam_idx1 : first camera id
 * cam_idx2 : second camera id
 * 
 * corresp_map: A boolean map of same size as reconstruction matrix, it contains
 * information whether a perticular point at (x,y) in reconstruction has been 
 * computed or not. (true if computed and false if not computed.)
 * 
 * reconstruction: matrix which contains reconstructed points. Some of the points in
 * reconstruction matrix are uncomputed (can be decided with help of corresp_map).
 * 
 * color: A mtrix which contains color information for reconstructed points.
 * 
 * method: This parameter decides which method is used to compute the depth map.
 * 
 */

int DepthMapStereo::computeStereoCorrespondence(Mat& leftImage, Mat& rightImage, Mat& reconstruction, Mat& color, cv::Mat &R , cv::Mat &T , cv::Mat &K , cv::Mat &D  , int map_img_idx, std::vector< Eigen::Vector3f >& points, std::vector< Eigen::Vector3f >& colors , int method  )
{
  
  
  int SADWindowSize = 0, numberOfDisparities = 0;
  
  enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2 };
  int alg = STEREO_BM;
  
//   if(method < 2)
//     alg = method;
    
  std::vector< cv::Point2f > points1,points2;
  std::vector< cv::Point3f > _object_points;
  
  double r1[9],r2[9],t1[3],t2[3],r[9],t[3],r1_inv[9],t1_inv[3];    
   
//   cv::gpu::
    
  int color_mode = alg == STEREO_BM ? 0 : -1;  
    
  cv::Size img_size = leftImage.size();
  
  cv::Mat lImage , rImage;
  
  if( alg == STEREO_BM )
  {
    cv::cvtColor( leftImage , lImage , cv::COLOR_RGB2GRAY );
    cv::cvtColor( rightImage , rImage , cv::COLOR_RGB2GRAY );
  }
  else
  {
    leftImage = lImage;
    rightImage = rImage;
  }
  

    
  cv::Rect roi1, roi2;
  cv::Mat Q;

        
  cv::Mat  R1, P1, R2, P2; //(3,3,CV_64FC1) (3,1,CV_64FC1)
    
  cv::Mat M1(3,3,CV_64FC1,K.data),M2(3,3,CV_64FC1,K.data),D1(5,1,CV_64FC1,D.data),D2(5,1,CV_64FC1,D.data);
  
        
  cv::stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY , 1.0 , img_size, &roi1, &roi2 );
        
  Mat map11, map12, map21, map22;
  cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
  cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
        
  cv::Mat img1r, img2r,img1r_trans,img2r_trans;
  cv::remap( lImage, img1r, map11, map12, INTER_LINEAR);
  cv::remap( rImage, img2r, map21, map22, INTER_LINEAR);
    
  cv::Mat image1,image2;
    
  image1 = leftImage;
  image2 = rightImage;
  
  cv::Mat img1 , img2;
  
  img1 = img1r;
  img2 = img2r;
    
    
  bool x_change = true;
    
  bool reverse = false;
    
  double cx_l = P1.at<double>(0,2);
  double cx_r = P2.at<double>(0,2);
  double cy_l = P1.at<double>(1,2);
  double cy_r = P2.at<double>(1,2);
  
  std::cout<<P1.at<double>(0,3)<<" "<<P1.at<double>(1,3)<<" "<<P2.at<double>(0,3)<<" "<<P2.at<double>(1,3)<<std::endl;
    
  if(((int)P2.at<double>(1,3))!=0)
    x_change = false;   
   
  if(!x_change)
  {
   transposeImage(img1r,img1r_trans);
   transposeImage(img2r,img2r_trans);
   
   img1 = img1r_trans;
   img2 = img2r_trans;
  }
  
  numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : (img_size.width/128)*16;
   
  Mat disp,disp8;
   
  if( alg == TR_STEREO_BM )
  {  
    bm.state->numberOfDisparities = numberOfDisparities;    
    bm(img1, img2, disp);   
    
    
    
//     cv:gpu::GpuMat gImag1 , gImag2 , gDisp;
//     
//     
//     cv::gpu::StereoBM_GPU bmgpu;//( 0 , numberOfDisparities );
//     
//     gImag1.upload( img1 );
//     gImag2.upload( img2 );
//     
//     bmgpu( gImag1 , gImag2 , gDisp );
//     
//     gDisp.download( disp );
//     
//     disp *= 16;
    
  }
  else
  {
    clock_t _begin_time = clock();
    int cn = img1.channels(); 
    sgbm.minDisparity = 10;
    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 16*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;    
    sgbm.numberOfDisparities = numberOfDisparities;    
    sgbm(img1, img2, disp);

  }
  
  disp.convertTo( disp8, CV_8U, 255/(numberOfDisparities*16.));

  
  cv::imshow( "disparity" , disp8 );
//   cv::waitKey();
  

  int disparityScale = 16; 
   
  if(x_change)
  {
   
   for(int y = 0; y < disp.rows; y++)
    {
      for(int x = 0; x < disp.cols; x++)
       {
         int d = disp.at<short>(y,x);
	  
	 d = d - disparityScale * (cx_l - cx_r);
	   
	 disp.at<short>(y,x) = d;
	  
	}
     }
   }
   else
   {
   
    for(int y = 0; y < disp.rows; y++)
     {
       for(int x = 0; x < disp.cols; x++)
        {
	  int d = disp.at<short>(y,x);

	  d = d - disparityScale * (cy_l - cy_r);
	   
	  disp.at<short>(y,x) = d;
	  
	}
     }   
   
   }
   
     if(x_change)
       std::cout<<"horizontal paralax "<<map_img_idx<<std::endl;
     else
       std::cout<<"vertical paralax "<<map_img_idx<<std::endl;
   
     Mat xyz; //reconstruction matrix
     
     
    reprojectImageTo3D(disp, xyz, Q, true); //
    
    
    Mat window1,window2;
	
    reconstruction = Mat(xyz.rows,xyz.cols,xyz.type());
    color = Mat(xyz.rows,xyz.cols,xyz.type());
	
    const double max_z = 1000;
	
    short *ptr2,*ptrl,*ptrr, *ptr_col;
	
    std::vector< cv::Vec3f > obj_pts;
    std::vector< cv::Point3f> obj_pts_pt3f;
    std::vector< cv::Vec2f > img_pts1,img_pts2;
    std::vector< int > colors_temp;
	
	if(map_img_idx==0)
          ptr2 = (short *)map11.data;
	else
	  ptr2 = (short *)map21.data;
	
	ptrl = (short *)map11.data;
	ptrr = (short *)map21.data;
	int pt_cnt = 0;
	
	int h = xyz.rows;
	int w = xyz.cols;
	
	Mat ref = leftImage;
	

	
	if(x_change)
	{
	
	for(int y = 0; y < xyz.rows; y++)
         {
          for(int x = 0; x < xyz.cols; x++)
           {
	      Vec3f point = xyz.at<Vec3f>(y, x);
	      
	      int d = disp.at<short>(y,x);
	      
	      d = d/16 + (cx_l - cx_r) ;
	      
	      uchar *ptr;
	      if(map_img_idx==0)
	      {
	       ptr = (uchar *)(img1.data + y * img1.step);
	      }
	      else
	      {
	       ptr = (uchar *)(img2.data + y * img2.step);
	      }
	      
	      
	      
	    cv::Vec3f color_val;
	      
	    int x_p,y_p,x1,y1,x2,y2;
	      
	    x1 = *(ptrl + y * map11.step/2 + 2*x);
            y1 = *(ptrl + y * map11.step/2 + 2*x + 1);
	      
	    x2 = *(ptrr + y * map21.step/2 + 2*(x - d));
            y2 = *(ptrr + y * map21.step/2 + 2*(x -d) + 1);
	      
	    uchar *ptr1 = (uchar *)(ref.data + ((int)y1) * ref.step);
	      
	    bool check = x1>5&&y1>5&&x2>5&&y2>5&&x1<(w-5)&&y1<(h-5)&&x2<(w-5)&&y2<(h-5);
	      
	      
	    double p2[] = { x2 , y2 , 1 },epl[3];
	    
            if(!check)
	      continue;
	      
	      
	    if(map_img_idx==0)
	    {
		
	        x_p = *(ptr2 + y * map11.step/2 + 2*x);
                y_p = *(ptr2 + y * map11.step/2 + 2*x + 1);
		
		if(d==0 )
	        {	      
// 	          corresp_map[y_p * xyz.cols + x_p] = false;
	          continue;
	        } 
		
		color_val.val[0] = ptr1[3*x1];
	        color_val.val[1] = ptr1[3*x1 + 1];
	        color_val.val[2] = ptr1[3*x1 + 2];
		
// 		match_list[x_p][2*y_p] = x2;
// 		match_list[x_p][2*y_p + 1] = y2;
		

	      }
	      else
	      {
	        x_p = *(ptr2 + y * map21.step/2 + 2*(x - d));
                y_p = *(ptr2 + y * map21.step/2 + 2*(x -d) + 1);
		
		if(d==0 )
	        {	      
// 	          corresp_map[y_p * xyz.cols + x_p] = false;
	          continue;
	        }
		
                color_val.val[0] = ptr1[3*x1];
	        color_val.val[1] = ptr1[3*x1 + 1];
	        color_val.val[2] = ptr1[3*x1 + 2];
		
// 		match_list[x_p][2*y_p] = x1;
// 		match_list[x_p][2*y_p + 1] = y1;
		

	      }
	      
              if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
	      { 
// 		if((y_p<xyz.rows)&&(x_p<xyz.cols)&&(y_p>0)&&(x_p>0))
// 		corresp_map[y_p * xyz.cols + x_p] = false;
		continue;
	      }
	      if(((y_p<xyz.rows)&&(x_p<xyz.cols)&&(y_p>0)&&(x_p>0)))
	      {		
	        reconstruction.at<Vec3f>(y_p,x_p) = point;
// 	        corresp_map[y_p * xyz.cols + x_p] = true;
	        color.at<Vec3f>(y_p,x_p) = color_val;
		
		points.push_back( Eigen::Vector3f( point[ 0 ] , point[ 1 ] , point[ 2 ]  ) );
		
		colors.push_back( Eigen::Vector3f( color_val[ 2 ] / 255.0 , color_val[ 1 ] / 255.0 , color_val[ 0 ] / 255.0 ) );
	       
	        pt_cnt++;
		

	      }
	   }
	 }
      }
    else
    {
      
     std::cout<<xyz.rows<<" "<<w<<std::endl;
     
     w = xyz.rows;
     
     h = xyz.cols;
    
           
     for(int y = 0; y < xyz.rows; y++)
      {
        for(int x = 0; x < xyz.cols; x++)
        {
            Vec3f point = xyz.at<Vec3f>(y, x);
	    int d = disp.at<short>(y,x);
	    
	    d = d/16 + (cy_l - cy_r) ;
	    
            uchar *ptr;
	    
	    
	      if(map_img_idx==0)
	      {
	       ptr = (uchar *)(img1.data + y * img1.step);
	      }
	      else
	      {
	       ptr = (uchar *)(img2.data + y * img2.step);
	      }
	      Vec3f color_val;
	      
	      
	      int x_p,y_p,x1,y1,x2,y2;
	      
	      x1 = *(ptrl + x * map11.step/2 + 2*y);
              y1 = *(ptrl + x * map11.step/2 + 2*y + 1);
	      
	      x2 = *(ptrr + x * map21.step/2 + 2*(y - d));
              y2 = *(ptrr + x * map21.step/2 + 2*(y - d) + 1);
	      
	      uchar *ptr1 = (uchar *)(ref.data + ((int)y1) * ref.step);
	      
	    bool check = x1 > 5 && y1 > 5 && x2 > 5 && y2 > 5 && x1 < (w-5) && y1 < (h-5) && x2 < (w-5) && y2 < (h-5);
	      
	    double p2[] = {x2,y2,1},epl[3];
	    
// 	    matrix_product(1,3,3,3,p2,(double *)F.data,epl);
// 	    
// 	    double distance = epl[0] * x1 + epl[1] * y1 + epl[2];
// 	    
// 	    distance /= sqrt(epl[0] * epl[0] + epl[1] * epl[1]);
	    
	    //distance = 0;
	    
	    if(!check)
	      continue;

	      if(map_img_idx==0)
	      {
	        x_p = *(ptr2 + x * map11.step/2 + 2*y);
                y_p = *(ptr2 + x * map11.step/2 + 2*y + 1);
		
	        if(d==0 )
	        {	      
// 	          corresp_map[y_p * w + x_p] = false;
	          continue;
	        } 
		
		color_val.val[0] = ptr1[ 3 * x1];
	        color_val.val[1] = ptr1[ 3 * x1 + 1];
	        color_val.val[2] = ptr1[ 3 * x1 + 2];
		
// 		match_list[x_p][2*y_p] = x2;
// 		match_list[x_p][2*y_p + 1] = y2;

	      }
	      else
	      {
	        x_p = *(ptr2 + x * map21.step/2 + 2*(y - d));
                y_p = *(ptr2 + x * map21.step/2 + 2*(y - d) + 1);
		
		if(d==0 )//|| abs(distance)>6
	        {	      
// 	          corresp_map[y_p * w + x_p] = false;
	          continue;
	        } 
		
		color_val.val[0] = ptr1[3*x1];
	        color_val.val[1] = ptr1[3*x1 + 1];
	        color_val.val[2] = ptr1[3*x1 + 2];
		
// 		match_list[x_p][2*y_p] = x1;
// 		match_list[x_p][2*y_p + 1] = y1;
		
	      }
	      
              if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
	      { 

// 		corresp_map[y_p * w + x_p] = false;
		continue;
	      }
	      if(((y_p<h)&&(x_p<w)&&(y_p>0)&&(x_p>0)))
	      {
		
	        window1 = image1( Rect(x1, y1, 5, 5) );
	        window2 = image2(Rect(x2, y2, 5, 5));
	       
	        Mat res(1,1,CV_32FC1);
	        reconstruction.at< cv::Vec3f >(y_p,x_p) = point;
// 	        corresp_map[y_p * w + x_p] = true;
	        color.at<Vec3f>(y_p,x_p) = color_val;
		
				points.push_back( Eigen::Vector3f( point[ 0 ] , point[ 1 ] , point[ 2 ]  ) );
		
		colors.push_back( Eigen::Vector3f( color_val[ 2 ] / 255.0 , color_val[ 1 ] / 255.0 , color_val[ 0 ] / 255.0 ) );
	       
	        pt_cnt++;


	      }

        }
    }
    
    
  }

  std::cout<<"total reconstructions "<<pt_cnt<<std::endl;  
  return pt_cnt;

}
 
 

 
 
 
 
}

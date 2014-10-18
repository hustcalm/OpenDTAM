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


#ifndef DEPTHMAPSTEREO_H
#define DEPTHMAPSTEREO_H

#include "external_includes/opencvincludes.h"
#include "external_includes/eigenincludes.h"
#include "drawimage.h"


namespace tr{

class DepthMapStereo
{
  

  
  cv::StereoSGBM sgbm;
  cv::StereoBM bm;
  
  vc::DrawImage *mDrawImage;
  
public:
  
  static const int TR_STEREO_BM = 0;
  static const int TR_STEREO_SGBM = 1;
  static const int TR_STEREO_BP = 2;
  static const int TR_STEREO_GC = 3;
  static const int TR_STEREO_BP_CONST_SPACE = 4;
  
  DepthMapStereo();
  
 
 /*
  * The method computes dense correspondence between two images. It also computes 3d reprojection
  * of dense correspondenses. 
  */ 
 
 
 int computeStereoCorrespondence(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat& reconstruction, cv::Mat& R, cv::Mat& T, cv::Mat& K, cv::Mat& D, cv::Mat& color, int map_img_idx, int method);
 
  /*
  * The method computes dense correspondence between two images. 
  *  
  */
 
 int computeStereoCorrespondence(cv::Mat& leftImage, cv::Mat& rightImage, cv::Mat& reconstruction, cv::Mat& color, cv::Mat& R, cv::Mat& T, cv::Mat& K, cv::Mat& D, int map_img_idx, std::vector< Eigen::Vector3f >& points, std::vector< Eigen::Vector3f >& colors, int method = TR_STEREO_SGBM );
 
 
 /*
  * The method computes dense correspondence between two images. 
  * 
  *TODO: remove this method (put it under research code.)
  */
 
 int computeStereoCorrespondenceIVT(int cam_idx1, int cam_idx2, 
				 std::vector< bool > &corresp_map, 
				 int** match_list, cv::Mat &color, 
				 int map_img_idx, 
				 int method = DepthMapStereo::TR_STEREO_BM);
 
 /**/
 
 void computeRelativePose(int camId1, int camId2, cv::Mat &rVec, cv::Mat &tVec);
 
 
/* This method transposes an image. It's useful in case of vertical paralax stereo.*/ 
 
 void transposeImage(cv::Mat &src, cv::Mat &dst);
 
 /*
  * This method is used to display 3d points with colors. This method uses vtk as display engine.
  * 
  */
 void displayReconstruction(cv::Mat &reconstruction, cv::Mat &color, std::vector<bool> corresp_map);
 void displayReconstruction(int camId1, int camId2);
 void displayReconstruction();
 
 /*This method is used to apply 3d transform to 3d points.*/
 
 void applyTransform(std::vector< cv::Vec3f > &points,double rotation[9],double translation[3]);
 
  /*This method is used to apply 3d transform to 3d points.*/
 
 void applyTransform(cv::Mat &points,double rotation[9],double translation[3]);
 
 /* Retrieve Correspondence */
 
 void retrieveCorrespondence(cv::Mat &depthImage, cv::Size imageSize1, 
			     cv::Size imageSize2,cv::Mat &matchList,cv::Mat map1x,
			     cv::Mat map1y,cv::Mat map2x, cv::Mat map2y,
			     std::vector<bool> &corresp_map, int paralaxType);
 
 void computePose(int camId, int neighborId, cv::Mat& rotation, cv::Mat& translation);
 
 void computeDenseTrack();
 
 void setDrawImage( vc::DrawImage *drawImage );
};

}

#endif // DEPTHMAPSTEREO_H

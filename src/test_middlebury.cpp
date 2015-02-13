/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2014, Itseez Inc, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Itseez Inc or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/


#include <opencv2/core/core.hpp>
#include <cstdio>
#include <string>
#include <vector>

// custom includes
#include "dataset/msm_middlebury.hpp"
#include "matching_reproject/stereo_matching.hpp"
#include "utils/util.hpp"

// Include logging facilities
#include "logger/log.h"
#ifndef FILELOG_MAX_LEVEL
    #define FILELOG_MAX_LEVEL logDEBUG4
#endif
#define FILE_LOG(level) \
  if (level > FILELOG_MAX_LEVEL) ;\
  else if (level > FILELog::ReportingLevel() || !Output2FILE::Stream()) ; \
   else FILELog().Get(level)



using namespace std;
using namespace cv;
using namespace cv::datasets;
using namespace stereo;

int main(int argc, char *argv[])
{

    FILE_LOG(logINFO) << "Binocular Dense Stereo";

    string path("../dataset/dataset_templeRing/");

    Ptr<MSM_middlebury> dataset = MSM_middlebury::create();
    dataset->load(path);

    // dataset contains camera parameters for each image.
    printf("images number: %u\n", (unsigned int)dataset->getTrain().size());

    Mat img1;
    Mat img2;

    // images path
    std::string img1_path = "../dataset/dataset_templeRing/templeR0001.png";
    std::string img2_path = "../dataset/dataset_templeRing/templeR0002.png";
    // load images data
    Ptr<MSM_middleburyObj> data_img1 = static_cast< Ptr<MSM_middleburyObj> >  (dataset->getTrain()[0]);
    Ptr<MSM_middleburyObj> data_img2 = static_cast< Ptr<MSM_middleburyObj> >  (dataset->getTrain()[1]);
    // load images
    stereo::loadImages(img1_path, img2_path, img1,img2);

    // init
    Mat R1,R2,P1,P2,Q;

    // zero distiorsions
    Mat D1 = Mat::zeros(1, 5, CV_64F);
    Mat D2 = Mat::zeros(1, 5, CV_64F);

    // load K and R from dataset info
    Mat M1 = Mat(data_img1->k);
    Mat M2 = Mat(data_img2->k);
    Mat r1 = Mat(data_img1->r);
    Mat r2 = Mat(data_img2->r);

    // init translation vectors from dataset
    Mat t1 = Mat(3, 1, CV_64FC1, &t1);
    Mat t2 = Mat(3, 1, CV_64FC1, &t2);

    // rotation between img2 and img1
    Mat R = r2*r1.t();
    // translation between img2 and img1
    Mat T = t1 - (R.t()*t2 );

    Rect roi1,roi2;
    stereo::rectifyImages(img1, img2, M1, D1, M2, D2, R, T, R1, R2, P1, P2, Q, roi1, roi2, 1.f);

//    util::infoMatrix(R);

    Mat disp;
    stereo::computeDisparity(img1, img2, disp,1);
    stereo::display(img1, img2, disp);

//    stereo::storePointCloud(disp, Q, "sblinda", mat);
//    stereo::storePointCloud(<#(Mat&)disp#>, <#(Mat&)Q#>, <#(const char*)filename#>, <#(const Mat&)mat#>);


    return 0;
}

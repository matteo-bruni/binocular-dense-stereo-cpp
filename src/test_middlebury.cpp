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

#include "dataset/msm_middlebury.hpp"


#include "matching_reproject/stereo_matching.hpp"



using namespace std;
using namespace cv;
using namespace cv::datasets;
using namespace stereo;

int main(int argc, char *argv[])
{
//    const char *keys =
//            "{ help h usage ? |    | show this message }"
//            "{ path p         |true| path to dataset description }";
//    CommandLineParser parser(argc, argv, keys);
//    string path(parser.get<string>("path"));
//    if (parser.has("help") || path=="true")
//    {
//        parser.printMessage();
//        return -1;
//    }
    string path("../dataset/dataset_templeRing/");

    Ptr<MSM_middlebury> dataset = MSM_middlebury::create();
    dataset->load(path);

    // ***************
    // dataset contains camera parameters for each image.
    printf("images number: %u\n", (unsigned int)dataset->getTrain().size());

    // For example, let output number of elements and last element.
//    Ptr<MSM_middleburyObj> example = static_cast< Ptr<MSM_middleburyObj> >  (dataset->getTrain().back());

//    printf("last image name: %s\n", (path + example->imageName).c_str());
//    printf("K:\n");
//    for (int i=0; i<3; ++i) {
//            for (int j=0; j<3; ++j) {
//                printf("%f ", example->k(i, j));
//            }
//            printf("\n");
//    }
//    printf("R:\n");
//    for (int i=0; i<3; ++i) {
//        for (int j=0; j<3; ++j) {
//            printf("%f ", example->r(i, j));
//        }
//        printf("\n");
//    }
//    printf("t:\n");
//    for (int i=0; i<3; ++i) {
//        printf("%f ", example->t[i]);
//    }
//    printf("\n");


    Ptr<MSM_middleburyObj> data_img1 = static_cast< Ptr<MSM_middleburyObj> >  (dataset->getTrain()[0]);

    Ptr<MSM_middleburyObj> data_img2 = static_cast< Ptr<MSM_middleburyObj> >  (dataset->getTrain()[1]);

    Mat img1;
    Mat img2;

    std::string img1_path = "../dataset/dataset_templeRing/templeR0001.png";
    std::string img2_path = "../dataset/dataset_templeRing/templeR0002.png";

    stereo::loadImages(img1_path, img2_path, img1,img2);

    Mat D1,D2,R1,R2,P1,P2,Q;

    Mat M1 =Mat(data_img1->k);
    Mat M2 =Mat(data_img2->k);
    Mat r1 = Mat(data_img1->r);
    Mat r2 = Mat(data_img2->r);

    Mat t1 = Mat(data_img1->t);
    Mat t2 = Mat(data_img2->t);

//    double t1[3];
//    std::copy(t1, data_img1->t,t1);
//    double t2[3];
//    std::copy(t2, data_img2->t,t2);
    Mat R = r2*r1.t();
    Mat T = t1 - (R.t()*t2);

//    Rect roi1,roi2;
//
//    stereo::rectifyImages(img1, img2, M1, D1, M2, D2, R, T, R1, R2, P1, P2, Q, roi1, roi2, 1.f);
//
//    Mat disp;
//
//    stereo::computeDisparity(img1, img2, disp,1);
//
//    stereo::display(img1, img2, disp);


    //stereo::storePointCloud(<#(Mat&)disp#>, <#(Mat&)Q#>, <#(const char*)filename#>, <#(const Mat&)mat#>);


    return 0;
}

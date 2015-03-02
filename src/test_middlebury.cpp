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

// include mik
#include <cstdint>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <string>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include <boost/thread/thread.hpp>
#include <stdint.h>


#include <pcl/common/projection_matrix.h>

// custom includes
#include "dataset/msm_middlebury.hpp"
#include "matching_reproject/stereo_matching.hpp"
#include "utils/util.hpp"
#include "stereo_viewer/viewer.hpp"
#include "registration/registration.hpp"

// Include logging facilities
#include "logger/log.h"

/////////////////////////////

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>


#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>


#ifndef FILELOG_MAX_LEVEL
    #define FILELOG_MAX_LEVEL logDEBUG4
#endif


using namespace std;
using namespace cv;
using namespace cv::datasets;


int main(int argc, char *argv[])
{

    FILE_LOG(logINFO) << "Binocular Dense Stereo";

    string path("../dataset/dataset_templeRing/");

    Ptr<MSM_middlebury> dataset = MSM_middlebury::create();
    dataset->load(path);

    // dataset contains camera parameters for each image.
    FILE_LOG(logINFO) << "images number: " << (unsigned int)dataset->getTrain().size();

    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
//
//    stereo::createAllClouds(dataset,clouds);
//    stereo_registration::registerClouds(clouds);


//    viewPointCloud(final_cloud);

  //  surfaceReconstruction();

//    //TEST SINGLE CLoUD
    int img1_num = 6;
    int img2_num = 7;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = stereo::generatePointCloud(dataset, 1, 2, true);
//    viewPointCloud(cloud1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = stereo::generatePointCloud(dataset, 3, 4, true);
//    viewPointCloud(cloud2);
   // stereo::viewDoublePointCloud(cloud1, cloud2);

    Ptr<cv::datasets::MSM_middleburyObj> data_img1 =
            static_cast< Ptr<cv::datasets::MSM_middleburyObj> >  (dataset->getTrain()[1]);
    Ptr<cv::datasets::MSM_middleburyObj> data_img2 =
            static_cast< Ptr<cv::datasets::MSM_middleburyObj> >  (dataset->getTrain()[3]);


    Mat r1 = Mat(data_img1->r);
    Mat r2 = Mat(data_img2->r);

    // init translation vectors from dataset
    Mat t1 = Mat(3, 1, CV_64FC1, &data_img1->t);
    Mat t2 = Mat(3, 1, CV_64FC1, &data_img2->t);

    // rotation between img2 and img1
    Mat R = r2*r1.t();
    // translation between img2 and img1
    Mat T = t1 - (R.t()*t2 );

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    transform_1 (0,0) = R.at<float>(0,0);
    transform_1 (0,1) = R.at<float>(0,1);
    transform_1 (0,2) = R.at<float>(0,2);

    transform_1 (1,0) = R.at<float>(1,0);
    transform_1 (1,1) = R.at<float>(1,1);
    transform_1 (1,2) = R.at<float>(1,2);
    transform_1 (2,0) = R.at<float>(2,0);
    transform_1 (2,1) = R.at<float>(2,1);
    transform_1 (2,2) = R.at<float>(2,2);

    transform_1 (0,3) = T.at<float>(0);
    transform_1 (1,3) = T.at<float>(1);
    transform_1 (2,3) = T.at<float>(2);




    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*cloud2, *transformed_cloud, transform_1);

    stereo::viewDoublePointCloud(transformed_cloud, cloud2);

//    cv::Mat result1 = stereo_util::segmentation(img1_num);
//
//    imshow("filtrata",result1);
//
//    cv::waitKey(0);


//    // ICP object.
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;
//    registration.setInputSource(cloud);
//    registration.setInputTarget(cloud2);
//
//    registration.align(*finalCloud);
//    if (registration.hasConverged())
//    {
//        std::cout << "ICP converged." << std::endl
//                << "The score is " << registration.getFitnessScore() << std::endl;
//        std::cout << "Transformation matrix:" << std::endl;
//        std::cout << registration.getFinalTransformation() << std::endl;
//    }
//    else std::cout << "ICP did not converge." << std::endl;
//
//
//    viewPointCloud(finalCloud);

    return 0;

}

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
#include "dataset/tsukuba_dataset.h"

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


using namespace std;
using namespace cv;
using namespace cv::datasets;


int main(int argc, char *argv[])
{

    FILE_LOG(logINFO) << "Binocular Dense Stereo";

//    string path("../dataset/dataset_templeRing_segm/");
//    Ptr<MSM_middlebury> dataset = MSM_middlebury::create();

    string path("../dataset/NTSD-200/");

    Ptr<tsukuba_dataset> dataset = tsukuba_dataset::create();
    dataset->load(path);

    // dataset contains camera parameters for each image.
    FILE_LOG(logINFO) << "images number: " << (unsigned int)dataset->getTrain().size();

//
//    cv::Mat img1, img2;
//
//    std::tuple<cv::Mat,cv::Mat> tuple_img = dataset->load_stereo_images(1);
//
//    // load images data
//    Ptr<cv::datasets::tsukuba_datasetObj> data_img1 =
//            static_cast< Ptr<cv::datasets::tsukuba_datasetObj> >  (dataset->getTrain()[1]);
//
//    Mat t1 = Mat(3, 1, CV_64FC1, &data_img1->tl);
//
//    Mat t2 = Mat(3, 1, CV_64FC1, &data_img1->tr);
//
//    FILE_LOG(logINFO) << "DATA: " << data_img1->imageName;
//
//    FILE_LOG(logINFO) << "DATA: " << data_img1->k << "\n" <<
//                data_img1->r << "\n" << t1 <<"\n" << t2;



    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    stereo::createAllCloudsTsukuba(dataset, clouds);



//    stereo_registration::registerClouds(clouds);


//    viewPointCloud(final_cloud);

  //  surfaceReconstruction();

////    //TEST SINGLE CLoUD
//    int img1_num = 6;
//    int img2_num = 7;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = stereo::generatePointCloud(dataset, 1, 2, true);
//    //    viewPointCloud(cloud1);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = stereo::generatePointCloud(dataset, 3, 4, true);
//    //    viewPointCloud(cloud2);
//    // stereo::viewDoublePointCloud(cloud1, cloud2);




    // qui
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud = clouds[0];

    for(int i = 1; i<clouds.size(); i++) {

            *finalCloud += *(clouds[i]);

    }
   stereo::viewPointCloud(finalCloud);











//    stereo::viewDoublePointCloud(clouds[0], clouds[1]);
//
//    FILE_LOG(logINFO) << "cloud1 : "<< cloud1->size();
//    FILE_LOG(logINFO) << "cloud2 : "<< transformed_cloud->size();
//
//    *cloud1 += *transformed_cloud;
//     stereo_registration::registerClouds(clouds);
//     stereo::viewPointCloud(finalCloud);
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

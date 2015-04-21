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


    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    int last_frame = 200;
    int step = 10;
    stereo::createAllCloudsTsukuba(dataset, clouds, last_frame, step);

    stereo_util::saveVectorCloudsToPLY(clouds, "original");

    FILE_LOG(logINFO) << "We have created: " << clouds.size() << " clouds from dataset. From images: ";
    for (int i=0; i<last_frame; i+=step){
        FILE_LOG(logINFO) << "image: " << i+1;
    }


    // TEST REGISTRATION  16 17
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr batch_cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::copyPointCloud(*clouds[16], *batch_cloud_sum);
//    *batch_cloud_sum += *(stereo_registration::naiveRegistration(clouds[17], clouds[16]));
//    stereo::viewPointCloud(batch_cloud_sum, " 16,17 ");

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr batch_cloud_sum2(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::copyPointCloud(*clouds[16], *batch_cloud_sum2);
//    *batch_cloud_sum += *(stereo_registration::naiveRegistration(clouds[17], clouds[16]));
//    stereo::viewPointCloud(batch_cloud_sum2, " 16,17 bis ");



    // TOTAL REGISTRATION
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_array = clouds;// = stereo_registration::register_clouds_in_batches(clouds, 2);
//    stereo::viewPointCloud(clouds[8], " cloud 15 - dopo");


//    stereo::viewDoublePointCloud(clouds[0],clouds[1], "cloud 0 e 1");
//    stereo::viewPointCloud(clouds_array[0], " step pre 0 - 0 e 1"); // ok
//
//    stereo::viewDoublePointCloud(clouds[2],clouds[3], "cloud 2 e 3");
//    stereo::viewPointCloud(clouds_array[1], " step pre 1 - 2 e 3"); // ok

//    stereo::viewDoublePointCloud(clouds[4], clouds[5], "cloud 4 e 5"); // ok
//    stereo::viewPointCloud(clouds_array[2], " step pre 2 - 4 e 5"); //ok
//
//    stereo::viewDoublePointCloud(clouds[6], clouds[7], "cloud 6 e 7");
//    stereo::viewPointCloud(clouds_array[3], " step pre 3 - 6 e 7"); //ok

//    stereo::viewDoublePointCloud(clouds[8], clouds[9], "cloud 8 e 9");
//    stereo::viewPointCloud(clouds_array[4], " step pre 4 - 8 e 9"); //fail


    int k = 0;
    while (clouds.size() != 1) {
        FILE_LOG(logINFO) << "Iterazione: " << k;
        clouds = stereo_registration::register_clouds_in_batches(clouds, 2);
        stereo_util::saveVectorCloudsToPLY(clouds, "register-step-"+std::to_string(k));
//        stereo::viewPointCloud(clouds_array[0], " step "+std::to_string(k));

        k++;
    }

    FILE_LOG(logINFO) << " post size :" << clouds[0]->size() << " ; total clouds = " << clouds.size();

    stereo::viewPointCloud(clouds[0]);



//SOMMA CON REGISTRAZIONE A BLOCCHI
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud = clouds[0];


//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr batch_cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);
//    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_array;
//
//    FILE_LOG(logINFO) << "single cloud size =  "<< clouds[0]->size();
//
//    int batch_size = 2;
//    unsigned long int n_batch = clouds.size()/batch_size + ((clouds.size() % batch_size != 0) ? 1:0);
//
//    FILE_LOG(logINFO) << "We have n_batch = : " << n_batch << " and n_clouds: "<< clouds.size();
//
//    for(int i = 0; i<n_batch; i++) {
//
//        FILE_LOG(logINFO) << "BATCH = : " << i << "from "<<  i*batch_size << " to: "<< min( int(clouds.size()), ((i+1)*batch_size));
//
//        int start = i*batch_size;
//        int stop = min( int(clouds.size()), ((i+1)*batch_size));
//
//        for(int j = start; j<stop; j++) {
//
//            if (j == start) {
//                FILE_LOG(logINFO) << j  <<" first cloud of the batch";
//                batch_cloud_sum = clouds[j];
//            }
//            else {
//                FILE_LOG(logINFO) << " registering "<< j << " with " << start;
//                *batch_cloud_sum += *(stereo_registration::naiveRegistration(clouds[j], clouds[start]));
//
//            }
//        }
//
//        clouds_array.push_back(batch_cloud_sum);
//        FILE_LOG(logINFO) << "BATCH point size =  "<< batch_cloud_sum->size();
//
//
//    }
//
//    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> registered_clouds_total;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr total_sum(new pcl::PointCloud<pcl::PointXYZRGB>);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//
//    temp_cloud = stereo_registration::naiveRegistration(clouds_array[0], clouds_array[1]);
//    for (int i=1; i<2; i++){
//
//        if (i == 0) {
//            FILE_LOG(logINFO) << i  <<" first cloud of the registration";
//            total_sum = clouds_array[0];
//        }
//        else {
//            FILE_LOG(logINFO) << " registering "<< i << " with " << 0;
//            *total_sum += *(stereo_registration::naiveRegistration(clouds_array[i], clouds_array[i-1]));
//
//        }
//    }
//    stereo::viewPointCloud(total_sum);





//    stereo::viewPointCloud(clouds_array[1]);
//    stereo::viewPointCloud(clouds_array[12]);

//    batch_cloud_sum =stereo_registration::registerClouds(clouds_array);

//
////
////


////REGISTRAZIONE NAIVE DUE A DUE
//
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum_registration = clouds[0];
//    for (int i=1; i<clouds.size(); i++){
//        cloud_sum_registration = stereo_registration::naiveRegistration(cloud_sum_registration, clouds[i]);
//        *cloud_sum_registration += *clouds[i];
//
//    }
//
//    stereo::viewPointCloud(cloud_sum_registration);


//SOLO SOMMA

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud = clouds[0];
//    for(int i = 0; i<clouds.size(); i++) {
//
//        *finalCloud += *(clouds[i]);
//
//    }
//    stereo::viewPointCloud(finalCloud);

////    //TEST SINGLE CLoUD
//    int img1_num = 6;
//    int img2_num = 7;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 = stereo::generatePointCloud(dataset, 1, 2, true);
//    //    viewPointCloud(cloud1);
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = stereo::generatePointCloud(dataset, 3, 4, true);
//    //    viewPointCloud(cloud2);
//    // stereo::viewDoublePointCloud(cloud1, cloud2);



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




    return 0;

}

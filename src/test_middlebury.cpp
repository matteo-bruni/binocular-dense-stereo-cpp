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
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <stdint.h>
#include <stdint-gcc.h>

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
using namespace pcl;

//
//
//boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
//{
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
//    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
//    viewer->addCoordinateSystem ( 1.0 );
//    viewer->initCameraParameters ();
//    return (viewer);
//}

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
    Mat t1 = Mat(3, 1, CV_64FC1, &data_img1->t);
    Mat t2 = Mat(3, 1, CV_64FC1, &data_img2->t);

    // rotation between img2 and img1
    Mat R = r2*r1.t();
    // translation between img2 and img1
    Mat T = t1 - (R.t()*t2 );

    Rect roi1,roi2;
    stereo::rectifyImages(img1, img2, M1, D1, M2, D2, R, T, R1, R2, P1, P2, Q, roi1, roi2, 1.f);

    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0,3);
    Q13 = Q.at<double>(1,3);
    Q23 = Q.at<double>(2,3);
    Q32 = Q.at<double>(3,2);
    Q33 = Q.at<double>(3,3);
//    util::infoMatrix(R);

    Mat disp;
    stereo::computeDisparity(img1, img2, disp,1);
    stereo::display(img1, img2, disp);
//    Mat recons3D;
//    stereo::storePointCloud(disp, Q, /*"sblinda",*/recons3D);
//
//
//
//    std::cout << "Creating Point Cloud..." <<std::endl;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
//
//    double px, py, pz;
//    uchar pr, pg, pb;
//
//    for (int i = 0; i < img1.rows; i++)
//    {
//        uchar* rgb_ptr = img1.ptr<uchar>(i);
//
//        uchar* disp_ptr = disp.ptr<uchar>(i);
//
//        double* recons_ptr = recons3D.ptr<double>(i);
//
//        for (int j = 0; j < img1.cols; j++)
//        {
//            //Get 3D coordinates
//
//            uchar d = disp_ptr[j];
//            if ( d == 0 ) continue; //Discard bad pixels
//            double pw = -1.0 * static_cast<double>(d) * Q32 + Q33;
//            px = static_cast<double>(j) + Q03;
//            py = static_cast<double>(i) + Q13;
//            pz = Q23;
//
//            px = px/pw;
//            py = py/pw;
//            pz = pz/pw;
//
//
//
//            //Get RGB info
//            pb = rgb_ptr[3*j];
//            pg = rgb_ptr[3*j+1];
//            pr = rgb_ptr[3*j+2];
//
//            //Insert info into point cloud structure
//            pcl::PointXYZRGB point;
//            point.x = px;
//            point.y = py;
//            point.z = pz;
//
//            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
//                    static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
//            point.rgb = *reinterpret_cast<float*>(&rgb);
//            point_cloud_ptr->points.push_back (point);
//        }
//    }
//    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
//    point_cloud_ptr->height = 1;
//
//    //Create visualizer
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    viewer = createVisualizer( point_cloud_ptr );
//
//    //Main loop
//    while ( !viewer->wasStopped())
//    {
//        viewer->spinOnce(100);
//        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }

    return 0;
}

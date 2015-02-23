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
#include <pcl/registration/icp.h>

#include <boost/thread/thread.hpp>
#include <stdint.h>
#include <stdint-gcc.h>

#include <pcl/common/projection_matrix.h>

// custom includes
#include "dataset/msm_middlebury.hpp"
#include "matching_reproject/stereo_matching.hpp"
#include "utils/util.hpp"

// Include logging facilities
#include "logger/log.h"

/////////////////////////////

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>


using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
class VoxelGrid;

#ifndef FILELOG_MAX_LEVEL
    #define FILELOG_MAX_LEVEL logDEBUG4
#endif

using namespace std;
using namespace cv;
using namespace cv::datasets;
using namespace stereo;
using namespace pcl;
using namespace stereo_util;



boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "reconstruction");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "reconstruction");
    viewer->addCoordinateSystem ( 1.0 );
    viewer->initCameraParameters ();
    return (viewer);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud(Ptr<MSM_middlebury> &dataset, const int img1_num, const int img2_num){

    Mat img1;
    Mat img2;

    // images path

    // load images data
    Ptr<MSM_middleburyObj> data_img1 = static_cast< Ptr<MSM_middleburyObj> >  (dataset->getTrain()[img1_num]);
    Ptr<MSM_middleburyObj> data_img2 = static_cast< Ptr<MSM_middleburyObj> >  (dataset->getTrain()[img2_num]);
    // load images
    stereo::loadImages(img1_num, img2_num, img1,img2);

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

//    double tx = atan2 (R.at<double>(3,2), R.at<double>(3,3));
//    double ty = - asin(R.at<double>(3,1));
//    double tz = atan2 (R.at<double>(2,1), R.at<double>(1,1));
//    FILE_LOG(logDEBUG) << "ROTATION " << img1_num << "-" <<img2_num<< " tx="<< tx <<" ty=" << ty << "tz= " << tz;

//    theta_x = arctan(r_{3,2}/r_{3,3})
//    \theta_y = -arcsin(r_{3,1})
//    \theta_z = arctan(r_{2,1}/r_{1,1})



    FILE_LOG(logINFO) << "Rectifying images...";
    Rect roi1,roi2;
    stereo::rectifyImages(img1, img2, M1, D1, M2, D2, R, T, R1, R2, P1, P2, Q, roi1, roi2, 1.f);


    FILE_LOG(logINFO) << "Computing Disparity map Dense Stereo";
    Mat disp(img1.size(), CV_32F);
    FILE_LOG(logINFO) << "imgsize " << stereo_util::infoMatrix(img1);
    FILE_LOG(logINFO) << "dispsize " << stereo_util::infoMatrix(disp);

    stereo::computeDisparity(img1_num, img2_num, img1, img2, disp,1,roi1,roi2);

   // stereo::display(img1, img2, disp);

    FILE_LOG(logINFO) << "Creating point cloud..";
    Mat recons3D(disp.size(), CV_32FC3);
    FILE_LOG(logINFO) << "recons3Dsize " << stereo_util::infoMatrix(recons3D);

    // stereo::storePointCloud(disp, Q, recons3D);

    //std::cout << "Creating Point Cloud..." <<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    stereo::createPointCloud(img1, img2, Q, disp, recons3D, point_cloud_ptr);

    return point_cloud_ptr;
}


void viewPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr) {
    //Create visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = createVisualizer( point_cloud_ptr );

    //Main loop
    while ( !viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void createAllClouds(Ptr<MSM_middlebury> &dataset, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds){


    int img1_num=1;
    int img2_num=2;


    std::stringstream ss;
    std::string path;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = generatePointCloud(dataset, img1_num, img2_num);
    clouds.push_back(cloud);

    pcl::io::savePCDFileASCII ("./cloud1.pcd", *cloud);
    unsigned int dataset_size = (unsigned int)dataset->getTrain().size();


    for(std::vector<std::tuple<int,int>>::iterator it = dataset->getAssociation().begin();
        it != dataset->getAssociation().end(); ++it) {
        img1_num = std::get<0>(*it);
        img2_num = std::get<1>(*it);
        cloud = generatePointCloud(dataset, img1_num, img2_num);
        if(!(*cloud).empty()){
            clouds.push_back(cloud);
            ss.str( std::string() );
            ss.clear();
            ss << img1_num<< "-" << img2_num ;
            path = "./cloud"+ ss.str() +".pcd";
            pcl::io::savePCDFileASCII (path, *cloud);
        }

        /* std::cout << *it; ... */
    }


//    // c'è un problema sull'ultima immagine
//    for (int i=2; i<dataset_size-1;i++){
//        img1_num = i;
//        img2_num = i+1;
//        cloud = generatePointCloud(dataset, img1_num, img2_num);
//        clouds.push_back(cloud);
//        if(!(*cloud).empty()){
//            ss.str( std::string() );
//            ss.clear();
//            ss <<  i;
//            path = "./cloud"+ ss.str() +".pcd";
//            pcl::io::savePCDFileASCII (path, *cloud);
//        }
//
//    }
    FILE_LOG(logINFO) << "cloud size" <<clouds.size();


}

void icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_sr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_tg,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out,const bool downsample,Eigen::Matrix4f &final_transform){

    ///downsampling
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<PointXYZRGB> grid;

    if (downsample)
    {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_sr);
        grid.filter (*src);

        grid.setInputCloud (cloud_tg);
        grid.filter (*tgt);
    }
    else
    {
        src = cloud_sr;
        tgt = cloud_tg;
    }


    // Compute surface normals and curvature
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new  pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new  pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZRGB,  pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    norm_est.setSearchMethod (tree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (src);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*src, *points_with_normals_src);

    norm_est.setInputCloud (tgt);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

    //run iterativatly icp

    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;

    //PARAMETRI DA SETTARE
    reg.setTransformationEpsilon (1e-8);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance (0.02);

    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);


    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;

    reg.setMaximumIterations (5); //era 2
    //i andava fino a 30
    for (int i = 0; i < 30; ++i)
    {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource (points_with_normals_src);
        reg.align (*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

    }

    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    //
    // Transform target back in source frame
    pcl::transformPointCloud (*cloud_tg, *cloud_out, targetToSource);

    //add the source to the transformed target
    *cloud_out += *cloud_sr;

    final_transform = targetToSource;

//    reg.align(*cloud_out);
//    std::cout << "ICP has converged = " << reg.hasConverged() <<endl;
//    // Save the transformed cloud
//    pcl::io::savePCDFileASCII ("cloud_after_icp.pcd", *cloud_out);
//    // Save the transformation
//    std::ofstream out2("transform_icp.txt");
//    Eigen::Affine3f Tr_icp;Tr_icp = reg.getFinalTransformation ();
//    cout<<"ICP transformation: "<<endl;
//    for(int i=0; i<4; i++){
//        for(int j=0; j<4; j++){
//            out2<<" "<<Tr_icp(i,j);
//            cout<<Tr_icp(i,j)<<"\t";
//        }
//        cout<<endl;
//        out2<<endl;
//    }
//    out2.close();

//    return cloud_out;
}

void registerClouds( std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds){

    unsigned int cloud_number = (unsigned int) clouds.size();
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i=1; i<cloud_number; i++){

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

        source = clouds[i-1];
        target = clouds[i];

        icp (source, target, temp, true, pairTransform);
        //transform current pair into the global transform
        pcl::transformPointCloud (*temp, *result, GlobalTransform);
        //update the global transform
        GlobalTransform = GlobalTransform * pairTransform;

        //save aligned pair, transformed into the first cloud's frame

    }

    std::stringstream ss;
    ss << "registrazione.pcd";
    pcl::io::savePCDFile (ss.str (), *result, true);
    viewPointCloud(result);

}

int main(int argc, char *argv[])
{

    FILE_LOG(logINFO) << "Binocular Dense Stereo";

    string path("../dataset/dataset_templeRing/");

    Ptr<MSM_middlebury> dataset = MSM_middlebury::create();
    dataset->load(path);

    // dataset contains camera parameters for each image.
    FILE_LOG(logINFO) << "images number: " << (unsigned int)dataset->getTrain().size();

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;

//    createAllClouds(dataset,clouds);
//
//    registerClouds(clouds);

//    viewPointCloud(final_cloud);



   //  TEST SINGLE CLoUD
    int img1_num = 1;
//    int img2_num = 2;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 = generatePointCloud(dataset, img1_num, img2_num);
//    viewPointCloud(cloud2);

    stereo_util::segmentation(img1_num);

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

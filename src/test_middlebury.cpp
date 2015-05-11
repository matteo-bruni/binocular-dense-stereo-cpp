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


#include "includes.h"

// custom includes
#include "dataset/msm_middlebury.hpp"
#include "matching_reproject/stereo_matching.hpp"
#include "utils/util.hpp"
#include "stereo_viewer/viewer.hpp"
#include "registration/registration.hpp"

// Include logging facilities
#include "logger/log.h"
#include "dataset/tsukuba_dataset.h"






using namespace std;
using namespace cv;
using namespace cv::datasets;


int main(int argc, char *argv[])
{

    FILE_LOG(logINFO) << "Binocular Dense Stereo";


    // string path("../dataset/dataset_templeRing_segm/");
    // Ptr<MSM_middlebury> dataset = MSM_middlebury::create();

    string path("../dataset/NTSD-200/");
    Ptr<tsukuba_dataset> dataset = tsukuba_dataset::create();
    dataset->load(path);
    // dataset contains camera parameters for each image.
    FILE_LOG(logINFO) << "images number: " << (unsigned int)dataset->getTrain().size();


    bool load_clouds = true;
    bool incremental = false;
    int load_n_clouds = 20;
    int first_frame = 0;
    int last_frame = 200;
    int step = 10;


    if (pcl::console::find_switch (argc, argv, "-load")) {

        pcl::console::parse (argc, argv, "-load", load_clouds);
        if (pcl::console::find_switch (argc, argv, "-n_clouds")){

            pcl::console::parse (argc, argv, "-n_clouds", load_n_clouds);

        }

    } else {

        if (pcl::console::find_switch (argc, argv, "-start")) {

            if (pcl::console::find_switch (argc, argv, "-end")) {

                pcl::console::parse (argc, argv, "-start", first_frame);
                pcl::console::parse (argc, argv, "-end", last_frame);

                if (pcl::console::find_switch (argc, argv, "-step"))
                    pcl::console::parse (argc, argv, "-step", step);

            }
        }
    }

    int cloud_num_1 = 0;
    int cloud_num_2 = 1;
    if (pcl::console::find_switch (argc, argv, "-incremental")) {
        pcl::console::parse (argc, argv, "-incremental", incremental);

        if (not incremental) {
            if (pcl::console::find_switch (argc, argv, "-cloud1")) {
                pcl::console::parse (argc, argv, "-cloud1", cloud_num_1);
            }
            if (pcl::console::find_switch (argc, argv, "-cloud2")) {
                pcl::console::parse (argc, argv, "-cloud2", cloud_num_2);
            }
        }
    }


    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    if (load_clouds) {

        clouds = stereo_util::loadVectorCloudsFromPLY("./original-", load_n_clouds);
        FILE_LOG(logINFO) << "Loading : " << load_n_clouds << " saved clouds";
    } else {

        FILE_LOG(logINFO) << "Generating clouds from frame : " << first_frame << " to frame " << last_frame <<
                            " with step " << step;
        stereo::createAllCloudsTsukuba(dataset, clouds, first_frame, last_frame, step);
        stereo_util::saveVectorCloudsToPLY(clouds, "original");

        FILE_LOG(logINFO) << "We have used: " << clouds.size() << " clouds from dataset. From images: ";
        for (int i=first_frame; i<last_frame; i+=step){
            FILE_LOG(logINFO) << "image: " << i+1;
        }

    }


    if (incremental) {
        // TOTAL REGISTRATION using incremental
        FILE_LOG(logINFO) << "Doing incremental registration ";

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud = stereo_registration::register_incremental_clouds(clouds);

        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        float leafSize = 2.0;
        grid.setLeafSize (leafSize, leafSize, leafSize);
        grid.setInputCloud (final_cloud);
        FILE_LOG(logINFO) << "finalcloud leafSize: " << leafSize<< " original size :" << final_cloud->size();
        grid.filter (*final_cloud);
        FILE_LOG(logINFO) << "finalcloud leafSize: " << leafSize<< " downsampled size :" << final_cloud->size();
        stereo::viewPointCloud(final_cloud);

        // END REGISTRATION using incremental
    } else {
        // TEST TWO CLOUD REGISTRATION
        FILE_LOG(logINFO) << "Doing clouds " << cloud_num_1 << " and " << cloud_num_2 << " registration ";


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr batch_cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*clouds[cloud_num_1], *batch_cloud_sum);
        stereo_registration::registrationParams pars;
        stereo_registration::CloudAlignment cloud_align = stereo_registration::registerSourceToTarget(clouds[cloud_num_2], clouds[cloud_num_1], pars);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_in_target_space(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud (*clouds[cloud_num_2], *cloud_source_in_target_space, cloud_align.transformMatrix);
        *batch_cloud_sum += *cloud_source_in_target_space;
        stereo::viewPointCloud(batch_cloud_sum, std::to_string(cloud_num_1)+" - "+std::to_string(cloud_num_2));

    }





    // TEST A COPPIE
//    int destination = 0;
//    int to_move = 0;
//    for (int i = 0; i<10 ; i ++){
//        destination =i*2;
//        to_move =i*2+1;
//
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr batch_cloud_sum2(new pcl::PointCloud<pcl::PointXYZRGB>);
//        pcl::copyPointCloud(*clouds[destination], *batch_cloud_sum2);
//        stereo_registration::CloudAlignment cloud_align2 = stereo_registration::registerSourceToTarget(clouds[to_move], clouds[destination]);
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_in_target_space2(new pcl::PointCloud<pcl::PointXYZRGB>);
//        pcl::transformPointCloud (*clouds[to_move], *cloud_source_in_target_space2, cloud_align2.transformMatrix);
//        *batch_cloud_sum2 += *cloud_source_in_target_space2;
//        stereo::viewPointCloud(batch_cloud_sum2, std::to_string(destination)+"-"+std::to_string(to_move));
//
//    }


//    // TOTAL REGISTRATION using batch
//    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_array = clouds;// = stereo_registration::register_clouds_in_batches(clouds, 2);
//    int k = 0;
//    while (clouds.size() != 1) {
//        FILE_LOG(logINFO) << "Iterazione: " << k;
//        clouds = stereo_registration::register_clouds_in_batches(clouds, 2);
////        stereo_util::saveVectorCloudsToPLY(clouds, "register-step-"+std::to_string(k));
////        stereo::viewPointCloud(clouds_array[0], " step "+std::to_string(k));
//
//        k++;
//    }
//    FILE_LOG(logINFO) << " post size :" << clouds[0]->size() << " ; total clouds = " << clouds.size();
////        stereo::viewPointCloud(clouds_array[0], " step "+std::to_string(k));
//    // END TOTAL REGISTRATION using batch





    return 0;

}

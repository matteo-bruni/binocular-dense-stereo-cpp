#ifndef BINOCULAR_DENSE_STEREO_REGISTRATION_HPP
#define BINOCULAR_DENSE_STEREO_REGISTRATION_HPP

#include "../includes.h"
#include "../stereo_viewer/viewer.hpp"

// Include logging facilities
#include "../logger/log.h"

namespace stereo_registration {

    struct CloudAlignment {
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        //std::string frame_name;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedCloud;
    };
    struct sacParams {
        double filter_limit = 1000.0;
        int max_sacia_iterations = 2000;
        double sac_max_correspondence_dist = 2000;
        double sac_min_correspondence_dist = 3;
    };
    struct registrationParams {
        float leaf_size = 6.5;
        int downsample_levels = 2;
        float downsample_decrease = 1.0;
        double normals_radius = 10;
        double features_radius = 150;

        sacParams sacPar;
    };

    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> register_clouds_in_batches(
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_register, int batch_size);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr register_incremental_clouds(
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_register);



    CloudAlignment registerSourceToTarget(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target,
                                          registrationParams params);





    //units are meters:
    const double FILTER_LIMIT = 1000.0;
    const int MAX_SACIA_ITERATIONS = 2000;
    const double NORMALS_RADIUS = 10;
    const double FEATURES_RADIUS = 150;
    const double SAC_MAX_CORRESPONDENCE_DIST = 2000;
    const double SAC_MIN_CORRESPONDENCE_DIST = 3;

    pcl::PointCloud<pcl::Normal>::Ptr getNormals( pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud, double normal_radius );
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures( pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double features_radius);
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>
            align( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                   sacParams params,
                   Eigen::Matrix4f init_transformation);

}

#endif
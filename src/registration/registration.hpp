#ifndef BINOCULAR_DENSE_STEREO_REGISTRATION_HPP
#define BINOCULAR_DENSE_STEREO_REGISTRATION_HPP

#include "../includes.h"
#include "../stereo_viewer/viewer.hpp"

// Include logging facilities
#include "../logger/log.h"

namespace binocular_dense_stereo {

    struct CloudAlignment {
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        //std::string frame_name;
        PointCloudRGB::Ptr alignedCloud;
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

    void printSacParams(sacParams pars);
    void printRegistrationParams(registrationParams pars);

    std::vector< PointCloudRGB::Ptr> register_clouds_in_batches(
            std::vector< PointCloudRGB::Ptr> clouds_to_register, int batch_size);

    PointCloudRGB::Ptr register_incremental_clouds(
            std::vector< PointCloudRGB::Ptr> clouds_to_register);



    CloudAlignment registerSourceToTarget(PointCloudRGB::Ptr cloud_source,
                                          PointCloudRGB::Ptr cloud_target,
                                          registrationParams params);





    //units are meters:
    const double FILTER_LIMIT = 1000.0;
    const int MAX_SACIA_ITERATIONS = 2000;
    const double NORMALS_RADIUS = 10;
    const double FEATURES_RADIUS = 150;
    const double SAC_MAX_CORRESPONDENCE_DIST = 2000;
    const double SAC_MIN_CORRESPONDENCE_DIST = 3;

    pcl::PointCloud<pcl::Normal>::Ptr getNormals( PointCloudRGB::Ptr incloud, double normal_radius );
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures( PointCloudRGB::Ptr incloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double features_radius);
    pcl::SampleConsensusInitialAlignment<PointTRGB, PointTRGB, pcl::FPFHSignature33>
            align( PointCloudRGB::Ptr cloud_target,
                   PointCloudRGB::Ptr cloud_source,
                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                   pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                   sacParams params,
                   Eigen::Matrix4f init_transformation);

}

#endif
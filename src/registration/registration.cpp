#include "../includes.h"

//class VoxelGrid;

// local include
#include "registration.hpp"
#include "../config/config.hpp"

namespace binocular_dense_stereo {

    PointCloudRGB::Ptr register_incremental_clouds(
            std::vector<PointCloudRGB::Ptr> clouds_to_register) {


        PointCloudRGB::Ptr final_cloud(new PointCloudRGB);
        PointCloudRGB::Ptr temp_cloud(new PointCloudRGB);

        PointCloudRGB::Ptr cloud_src;
        PointCloudRGB::Ptr cloud_tgt;
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();


        pcl::copyPointCloud(*clouds_to_register[0], *final_cloud);
        for (int i=0; i<clouds_to_register.size()-1; i++){
            cloud_src = clouds_to_register[i];
            cloud_tgt = clouds_to_register[i+1];
            FILE_LOG(logINFO) << "registering clouds: " << i << " to "<< i+1;
            registrationParams pars = binocular_dense_stereo::ConfigLoader::get_instance().loadRegistrationParams();
            CloudAlignment output = binocular_dense_stereo::registerSourceToTarget(cloud_src, cloud_tgt, pars);

            transformMatrix = output.transformMatrix;
            temp_cloud->clear();
            pcl::transformPointCloud (*final_cloud, *temp_cloud, transformMatrix);
            final_cloud->clear();
            pcl::copyPointCloud(*temp_cloud, *final_cloud);

            // visualization
//            local_cloud->clear();
//            pcl::transformPointCloud (*cloud_src, *local_cloud, transformMatrix);
//            *local_cloud += *cloud_tgt;
//            binocular_dense_stereo::viewPointCloud(local_cloud, "step "+std::to_string(i)+" - "+std::to_string(i+1));
            // end visualization

            *final_cloud += *cloud_tgt;
        }

        FILE_LOG(logINFO) << "Final cloud point size =  " << final_cloud->size();
        return final_cloud;
    }



    std::vector< PointCloudRGB::Ptr> register_clouds_in_batches(
            std::vector< PointCloudRGB::Ptr> clouds_to_register, int batch_size) {


        std::vector< PointCloudRGB::Ptr> output_batches_of_registered_clouds;

        FILE_LOG(logDEBUG) << "single cloud size :"<< clouds_to_register[0]->size();

        unsigned long int n_batch = clouds_to_register.size()/batch_size + ((clouds_to_register.size() % batch_size != 0) ? 1:0);

        FILE_LOG(logINFO) << "We have n_batch = : " << n_batch << " of size: " << batch_size << " from a total of : " << clouds_to_register.size() << "clouds";

        for(int i = 0; i<n_batch; i++) {

            int start = i*batch_size;
            int stop = std::min( int(clouds_to_register.size()), ((i+1)*batch_size));

            FILE_LOG(logINFO) << "BATCH = : " << i << " from "<<  start << " to: "<< stop;
            FILE_LOG(logINFO) << start  <<" first cloud of the batch";

            PointCloudRGB::Ptr batch_cloud_sum(new PointCloudRGB);

            for(int j = start; j < stop; j++) {

                if (j == start) {
                    //clouds_to_register[j]; //
                    pcl::copyPointCloud(*clouds_to_register[start], *batch_cloud_sum);

                }
                else {
                    FILE_LOG(logINFO) << " registering "<< j << " in " << start << " space";
                    registrationParams par;
                    CloudAlignment output = binocular_dense_stereo::registerSourceToTarget(clouds_to_register[j], clouds_to_register[start], par);
                    *batch_cloud_sum += *(output.alignedCloud);
                }
            }
            binocular_dense_stereo::viewPointCloudRGB(batch_cloud_sum, " SUM from  "+std::to_string(start)+" to "+std::to_string(stop));

            output_batches_of_registered_clouds.push_back(batch_cloud_sum);
            FILE_LOG(logINFO) << "BATCH sum point size =  "<< batch_cloud_sum->size();

        }
        FILE_LOG(logINFO) << "We have a total of :"<< output_batches_of_registered_clouds.size() << " batches";
        return output_batches_of_registered_clouds;

    }


    CloudAlignment registerSourceToTarget(PointCloudRGB::Ptr cloud_source,
                                          PointCloudRGB::Ptr cloud_target,
                                          registrationParams params) {
        // ICP object.
        PointCloudRGB::Ptr cloud_source_to_target_downsampled(new PointCloudRGB);
        PointCloudRGB::Ptr cloud_source_downsampled(new PointCloudRGB);
        PointCloudRGB::Ptr cloud_target_downsampled(new PointCloudRGB);

        float leafSize = params.leaf_size;
        int DOWNSAMPLE_LEVELS = params.downsample_levels;
        float downsample_decrease = params.downsample_decrease;

        pcl::VoxelGrid<PointTRGB> grid;
        Eigen::Matrix4f transformMatrix (Eigen::Matrix4f::Identity ());
        double score = -1.;

        for (int i = 0; i < DOWNSAMPLE_LEVELS; i++){

            cloud_source_downsampled->clear();
            cloud_target_downsampled->clear();

            if (i < DOWNSAMPLE_LEVELS) {
                // DOWNSAMPLE
                FILE_LOG(logINFO) << "STEP" << i << " leafSize: " << leafSize<< " original size :" << cloud_source->size() << " ; " << cloud_target->size();
                grid.setLeafSize (leafSize, leafSize, leafSize);
                grid.setInputCloud (cloud_source);
                grid.filter (*cloud_source_downsampled);
                grid.setInputCloud (cloud_target);
                grid.filter (*cloud_target_downsampled);
                FILE_LOG(logINFO) << " post size :" << cloud_source_downsampled->size() << " ; " << cloud_target_downsampled->size();
                // END DOWNSAMPLE
            } else {
                FILE_LOG(logINFO) << " last step without downsample";

                pcl::copyPointCloud(*cloud_source, *cloud_source_downsampled);
                pcl::copyPointCloud(*cloud_target, *cloud_target_downsampled);

            }

            // SIMPLE ICP
//            pcl::IterativeClosestPoint<PointT, PointT> registration;
//            registration.setInputSource(cloud_source_downsampled);
//            registration.setInputTarget(cloud_target_downsampled);
//            registration.setTransformationEpsilon (1e-8);
//            registration.setMaxCorrespondenceDistance (0.5);
//            registration.setRANSACIterations(2000);
////            registration.setMaximumIterations(1000);
////            registration.setEuclideanFitnessEpsilon(1e-5); //1);
//            registration.setRANSACOutlierRejectionThreshold(0.1);
//            registration.align(*cloud_source_to_target_downsampled, transformMatrix);
//            FILE_LOG(logINFO) << "Default reg values: ";
//            FILE_LOG(logINFO) << "Default getMaxCorrespondenceDistance: " << registration.getMaxCorrespondenceDistance();
//            FILE_LOG(logINFO) << "Default getRANSACIterations: " << registration.getRANSACIterations();
//            FILE_LOG(logINFO) << "Default getEuclideanFitnessEpsilon: " << registration.getEuclideanFitnessEpsilon();
//            if (registration.hasConverged())
//            {
//                FILE_LOG(logINFO) << "registration step " << i << " ICP converged." << "The score is " << registration.getFitnessScore();
//                if (score == -1)
//                    score = registration.getFitnessScore();                if (score >= registration.getFitnessScore()){
//                    transformMatrix = registration.getFinalTransformation();
//                    score = registration.getFitnessScore();
//                } else {
//                    FILE_LOG(logINFO) << "Skipping registration step, score increasing";
//                }
//
////            std::cout << "Transformation matrix:" << std::endl;
//                std::cout << registration.getFinalTransformation() << std::endl;
//            }
//            else FILE_LOG(logINFO) << "registration step " << i << "ICP did not converge.";

            if (params.use_sac){

                //compute normals
                pcl::PointCloud<pcl::Normal>::Ptr normals_source = getNormals( cloud_source_downsampled, params.normals_radius );
                pcl::PointCloud<pcl::Normal>::Ptr normals_target = getNormals( cloud_target_downsampled, params.normals_radius );

                //compute local features
                pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_source = getFeatures( cloud_source_downsampled,
                                                                                    normals_source, params.features_radius);
                pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_target = getFeatures( cloud_target_downsampled,
                                                                                    normals_target, params.features_radius);

                //Get an initial estimate for the transformation using SAC
                //returns the transformation for cloud2 so that it is aligned with cloud1
                pcl::SampleConsensusInitialAlignment<PointTRGB, PointTRGB, pcl::FPFHSignature33> sac_ia = align( cloud_target_downsampled, cloud_source_downsampled,
                                                                                                                               features_target, features_source, params.sacPar,transformMatrix);
//                Eigen::Matrix4f	init_transform = sac_ia.getFinalTransformation();
                if (sac_ia.hasConverged())
                {
                    FILE_LOG(logINFO) << " sac registration step " << i << " SAC converged." << "The score is " << sac_ia.getFitnessScore();
                    if (score == -1)
                        score = sac_ia.getFitnessScore();
                    if (score >= sac_ia.getFitnessScore()){
                        transformMatrix = sac_ia.getFinalTransformation();
                        score = sac_ia.getFitnessScore();
                    } else {
                        FILE_LOG(logINFO) << "Skipping sac registration step, score increasing";
                    }
                    std::cout << sac_ia.getFinalTransformation() << std::endl;
                }
                else FILE_LOG(logINFO) << "sac registration step " << i << "SAC did not converge.";

            } else {
                FILE_LOG(logINFO) << "skipping sac.";

            }

            pcl::IterativeClosestPoint<PointTRGB, PointTRGB> registration;
            registration.setInputSource(cloud_source_downsampled);
            registration.setInputTarget(cloud_target_downsampled);
            registration.setTransformationEpsilon (params.icpPar.TransformationEpsilon);
            registration.setMaxCorrespondenceDistance (params.icpPar.MaxCorrespondenceDistance);
            registration.setRANSACIterations(params.icpPar.RANSACIterations);
            registration.setMaximumIterations(params.icpPar.MaximumIterations);
            registration.setEuclideanFitnessEpsilon(params.icpPar.EuclideanFitnessEpsilon); //1);
            registration.setRANSACOutlierRejectionThreshold(params.icpPar.RANSACOutlierRejectionThreshold);
            FILE_LOG(logINFO) << "pre icp align.";
            registration.align(*cloud_source_to_target_downsampled, transformMatrix);
            FILE_LOG(logINFO) << "post icp align.";
            if (registration.hasConverged())
            {
                FILE_LOG(logINFO) << "registration step " << i << " ICP converged.";
//                FILE_LOG(logINFO) << "The score is " << registration.getFitnessScore();
//                if (score == -1)
//                    score = registration.getFitnessScore();
//                if (score >= registration.getFitnessScore()){
                    transformMatrix = registration.getFinalTransformation();
//                    score = registration.getFitnessScore();
//                } else {
//                    FILE_LOG(logINFO) << "Skipping registration step, score increasing";
//                }

                //            std::cout << "Transformation matrix:" << std::endl;
                std::cout << registration.getFinalTransformation() << std::endl;
            }
            else FILE_LOG(logINFO) << "registration step " << i << "ICP did not converge.";


            leafSize -= downsample_decrease;
        }

        CloudAlignment output;
        // Transform target back in source frame
        PointCloudRGB::Ptr cloud_source_to_target_output(new PointCloudRGB);
        pcl::transformPointCloud (*cloud_source, *cloud_source_to_target_output, transformMatrix);
        output.alignedCloud = cloud_source_to_target_output;
        output.transformMatrix = transformMatrix;
        return output;
    }


    //computes the transformation for cloud2 so that it is transformed so that it is aligned with cloud1
    pcl::SampleConsensusInitialAlignment<PointTRGB, PointTRGB, pcl::FPFHSignature33>
    align( PointCloudRGB::Ptr cloud_target,
           PointCloudRGB::Ptr cloud_source,
           pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
           pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
           sacParams params,
           Eigen::Matrix4f init_transformation) {

        pcl::SampleConsensusInitialAlignment<PointTRGB, PointTRGB, pcl::FPFHSignature33> sac_ia;
        Eigen::Matrix4f final_transformation;
        sac_ia.setInputSource(cloud_source);
        sac_ia.setSourceFeatures(source_features);
        sac_ia.setInputTarget(cloud_target);
        sac_ia.setTargetFeatures(target_features);
        sac_ia.setMaximumIterations( params.max_sacia_iterations );
        sac_ia.setMinSampleDistance (params.sac_min_correspondence_dist);
        sac_ia.setMaxCorrespondenceDistance (params.sac_max_correspondence_dist);
        PointCloudRGB finalcloud;
        sac_ia.align( finalcloud, init_transformation );
        sac_ia.getCorrespondenceRandomness();
        return sac_ia;
    }

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures( PointCloudRGB::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double features_radius ) {

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr features = pcl::PointCloud<pcl::FPFHSignature33>::Ptr (new  pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::search::KdTree<PointTRGB>::Ptr search_method_ptr = pcl::search::KdTree<PointTRGB>::Ptr (new pcl::search::KdTree<PointTRGB>);
        pcl::FPFHEstimation<PointTRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        fpfh_est.setInputCloud( cloud );
        fpfh_est.setInputNormals( normals );
        fpfh_est.setSearchMethod( search_method_ptr );
        fpfh_est.setRadiusSearch( features_radius );
        fpfh_est.compute( *features );
        return features;
    }

    pcl::PointCloud<pcl::Normal>::Ptr getNormals( PointCloudRGB::Ptr incloud, double normal_radius ) {

        pcl::PointCloud<pcl::Normal>::Ptr normalsPtr = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<PointTRGB, pcl::Normal> norm_est;
        norm_est.setInputCloud( incloud );
        norm_est.setRadiusSearch( normal_radius );
        norm_est.compute( *normalsPtr );
        return normalsPtr;
    }


    void printSacParams(sacParams pars) {

        FILE_LOG(logINFO) << "Sac Params: ";
        FILE_LOG(logINFO) << "  filter_limit: " << pars.filter_limit;
        FILE_LOG(logINFO) << "  max_sacia_iterations: " << pars.max_sacia_iterations;
        FILE_LOG(logINFO) << "  sac_max_correspondence_dist: " << pars.sac_max_correspondence_dist;
        FILE_LOG(logINFO) << "  sac_min_correspondence_dist: " << pars.sac_min_correspondence_dist;

    }

    void printRegistrationParams(registrationParams pars) {

        FILE_LOG(logINFO) << "Registration Params: ";
        FILE_LOG(logINFO) << "  leaf_size: " << pars.leaf_size;
        FILE_LOG(logINFO) << "  downsample_levels: " << pars.downsample_levels;
        FILE_LOG(logINFO) << "  downsample_decrease: " << pars.downsample_decrease;
        FILE_LOG(logINFO) << "  normals_radius: " << pars.normals_radius;
        FILE_LOG(logINFO) << "  features_radius: " << pars.features_radius;

        printSacParams(pars.sacPar);


    }
}


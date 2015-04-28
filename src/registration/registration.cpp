
// OLD
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
//

#include <fstream>
#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

//#include <pcl/io/pcd_io.h>
//#include <pcl/console/time.h>
//#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/registration/correspondence_estimation.h>
//#include <pcl/registration/correspondence_estimation_normal_shooting.h>
//#include <pcl/registration/correspondence_estimation_backprojection.h>
//#include <pcl/registration/correspondence_rejection_median_distance.h>
//#include <pcl/registration/correspondence_rejection_surface_normal.h>
//#include <pcl/registration/transformation_estimatiosrcn_point_to_plane_lls.h>
//#include <pcl/registration/default_convergence_criteria.h>

#include <pcl/visualization/pcl_visualizer.h>


class VoxelGrid;

// local include
#include "registration.hpp"
#include "../stereo_viewer/viewer.hpp"
//

// Include logging facilities
#include "../logger/log.h"
#include "../includes.h"

#include <algorithm>

namespace stereo_registration {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr register_incremental_clouds(
            std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_register) {


        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt;
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();


        pcl::copyPointCloud(*clouds_to_register[0], *final_cloud);
        for (int i=0; i<clouds_to_register.size()-1; i++){
            cloud_src = clouds_to_register[i];
            cloud_tgt = clouds_to_register[i+1];
            FILE_LOG(logINFO) << "registering clouds: " << i << " to "<< i+1;
            CloudAlignment output = stereo_registration::registerSourceToTarget(cloud_src, cloud_tgt);

            transformMatrix = output.transformMatrix;
            pcl::transformPointCloud (*final_cloud, *final_cloud, transformMatrix);

            local_cloud->clear();
            pcl::transformPointCloud (*cloud_src, *local_cloud, transformMatrix);
            *local_cloud += *cloud_tgt;
            stereo::viewPointCloud(local_cloud, "step "+std::to_string(i)+" - "+std::to_string(i+1));

            *final_cloud += *cloud_tgt;
        }

        FILE_LOG(logINFO) << "Final cloud point size =  " << final_cloud->size();
        return final_cloud;
    }



    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> register_clouds_in_batches(
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_register, int batch_size) {


        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> output_batches_of_registered_clouds;

        FILE_LOG(logDEBUG) << "single cloud size :"<< clouds_to_register[0]->size();

        unsigned long int n_batch = clouds_to_register.size()/batch_size + ((clouds_to_register.size() % batch_size != 0) ? 1:0);

        FILE_LOG(logINFO) << "We have n_batch = : " << n_batch << " of size: " << batch_size << " from a total of : " << clouds_to_register.size() << "clouds";

        for(int i = 0; i<n_batch; i++) {

            int start = i*batch_size;
            int stop = std::min( int(clouds_to_register.size()), ((i+1)*batch_size));

            FILE_LOG(logINFO) << "BATCH = : " << i << " from "<<  start << " to: "<< stop;
            FILE_LOG(logINFO) << start  <<" first cloud of the batch";

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr batch_cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);

            for(int j = start; j < stop; j++) {

                if (j == start) {
                    //clouds_to_register[j]; //
                    pcl::copyPointCloud(*clouds_to_register[start], *batch_cloud_sum);

                }
                else {
                    FILE_LOG(logINFO) << " registering "<< j << " in " << start << " space";
                    CloudAlignment output = stereo_registration::registerSourceToTarget(clouds_to_register[j], clouds_to_register[start]);
                    *batch_cloud_sum += *(output.alignedCloud);
                }
            }
            stereo::viewPointCloud(batch_cloud_sum, " SUM from  "+std::to_string(start)+" to "+std::to_string(stop));

            output_batches_of_registered_clouds.push_back(batch_cloud_sum);
            FILE_LOG(logINFO) << "BATCH sum point size =  "<< batch_cloud_sum->size();

        }
        FILE_LOG(logINFO) << "We have a total of :"<< output_batches_of_registered_clouds.size() << " batches";
        return output_batches_of_registered_clouds;

    }


    CloudAlignment registerSourceToTarget(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target) {
        // ICP object.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_to_target_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

        float leafSize = 6.5;
        int DOWNSAMPLE_LEVELS = 5;
        pcl::VoxelGrid<pcl::PointXYZRGB> grid;
        Eigen::Matrix4f transformMatrix (Eigen::Matrix4f::Identity ());
        double score = -1.;

        for (int i = 0; i < DOWNSAMPLE_LEVELS; i++){

            cloud_source_downsampled->clear();
            cloud_target_downsampled->clear();

            // DOWNSAMPLE
            FILE_LOG(logINFO) << "STEP" << i << " leafSize: " << leafSize<< " original size :" << cloud_source->size() << " ; " << cloud_target->size();
            grid.setLeafSize (leafSize, leafSize, leafSize);
            grid.setInputCloud (cloud_source);
            grid.filter (*cloud_source_downsampled);
            grid.setInputCloud (cloud_target);
            grid.filter (*cloud_target_downsampled);
            FILE_LOG(logINFO) << " post size :" << cloud_source_downsampled->size() << " ; " << cloud_target_downsampled->size();
            // END DOWNSAMPLE



            // Compute surface normals and curvature
            PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
            PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
            pcl::NormalEstimation<PointT, PointNormalT> norm_est;
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
            norm_est.setSearchMethod (tree);
            norm_est.setKSearch (30);
            norm_est.setInputCloud (cloud_source_downsampled);
            norm_est.compute (*points_with_normals_src);
            pcl::copyPointCloud (*cloud_source_downsampled, *points_with_normals_src);
            norm_est.setInputCloud (cloud_target_downsampled);
            norm_est.compute (*points_with_normals_tgt);
            pcl::copyPointCloud (*cloud_target_downsampled, *points_with_normals_tgt);

            // Instantiate our custom point representation (defined above) ...
            MyPointRepresentation point_representation;
            // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
            float alpha[4] = {1.0, 1.0, 1.0, 1.0};
            point_representation.setRescaleValues (alpha);
            // Align
            pcl::IterativeClosestPoint<PointNormalT, PointNormalT> registration;
//            registration.setTransformationEpsilon (1e-6);
            // Set the maximum distance between two correspondences (src<->tgt) to 10cm
            // Note: adjust this based on the size of your datasets
//            registration.setMaxCorrespondenceDistance (0.5);
            // Set the point representation
            registration.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));
            registration.setInputSource (points_with_normals_src);
            registration.setInputTarget (points_with_normals_tgt);
            PointCloudWithNormals::Ptr reg_result (new PointCloudWithNormals);
            registration.align(*reg_result, transformMatrix);



//            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;
//            registration.setInputSource(cloud_source_downsampled);
//            registration.setInputTarget(cloud_target_downsampled);
//            registration.setTransformationEpsilon (1e-8);
//            registration.setMaxCorrespondenceDistance (2.5);
//            registration.setRANSACIterations(2000);
//            registration.setMaximumIterations(1000);
//            registration.setEuclideanFitnessEpsilon(1e-5); //1);
//            registration.align(*cloud_source_to_target_downsampled, transformMatrix);

            if (registration.hasConverged())
            {
                FILE_LOG(logINFO) << "registration step " << i << " ICP converged." << "The score is " << registration.getFitnessScore();
                if (score == -1)
                    score = registration.getFitnessScore();

                if (score >= registration.getFitnessScore()){
                    transformMatrix = registration.getFinalTransformation();
                    score = registration.getFitnessScore();
                } else {
                    FILE_LOG(logINFO) << "Skipping registration step, score increasing";
                }

//            std::cout << "Transformation matrix:" << std::endl;
                std::cout << registration.getFinalTransformation() << std::endl;
            }
            else FILE_LOG(logINFO) << "registration step " << i << "ICP did not converge.";

            leafSize -= 1;
        }

        CloudAlignment output;
        // Transform target back in source frame
        cloud_source_to_target_downsampled->clear();
        pcl::transformPointCloud (*cloud_source, *cloud_source_to_target_downsampled, transformMatrix);
        output.alignedCloud = cloud_source_to_target_downsampled;
        output.transformMatrix = transformMatrix;
        return output;
    }


}


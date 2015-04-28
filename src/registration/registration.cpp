
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

//using namespace std;
//using namespace pcl;
//using namespace pcl::io;
//using namespace pcl::console;
//using namespace pcl::registration;
//using namespace pcl::visualization;

//
pcl::PointCloud<pcl::PointNormal>::Ptr src, tgt;
//
bool rejection = true;
bool visualize = false;

//boost::shared_ptr<PCLVisualizer> vis;
class VoxelGrid;

// local include
#include "registration.hpp"
#include "../stereo_viewer/viewer.hpp"
//

// Include logging facilities
#include "../logger/log.h"

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
            transformMatrix = stereo_registration::naiveRegistrationTransformation(cloud_src, cloud_tgt);
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
                    *batch_cloud_sum += *(stereo_registration::naiveRegistrationCloud(clouds_to_register[j], clouds_to_register[start]));
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

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        float leafSize = 6.5;
        int DOWNSAMPLE_LEVELS = 5;
        pcl::VoxelGrid<pcl::PointXYZRGB> grid, grid2;
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        double score = -1.;

        for (int i = 0; i < DOWNSAMPLE_LEVELS; i++){
            // Downsample
            FILE_LOG(logINFO) << "STEP" << i << " leafSize: " << leafSize<< " original size :" << cloud_source->size() << " ; " << cloud_target->size();
            grid.setLeafSize (leafSize, leafSize, leafSize);
            grid.setInputCloud (cloud_source);
            grid.filter (*cloud_source_filtered);
            grid2.setLeafSize (leafSize, leafSize, leafSize);
            grid2.setInputCloud (cloud_target);
            grid2.filter (*cloud_target_filtered);
            FILE_LOG(logINFO) << " post size :" << cloud_source_filtered->size() << " ; " << cloud_target_filtered->size();

            pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;

            registration.setInputSource(cloud_source_filtered);
            registration.setInputTarget(cloud_target_filtered);

//            registration.setTransformationEpsilon (1e-8);
//            registration.setMaxCorrespondenceDistance (2.5);
            registration.setRANSACIterations(2000);
            registration.setMaximumIterations(1000);
//            registration.setEuclideanFitnessEpsilon(1e-5); //1);

            registration.align(*cloud_source_to_target_downsampled, transformMatrix);

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
        output.alignedCloud = cloud_source_to_target_downsampled;
        output.transformMatrix = transformMatrix;
        return output;
    }



    Eigen::Matrix4f naiveRegistrationTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target){


        //    // ICP object.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_to_target_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        FILE_LOG(logINFO) << " original size :" << cloud_source->size() << " ; " << cloud_target->size();
        pcl::VoxelGrid<pcl::PointXYZRGB> grid, grid2;
        grid.setLeafSize (3.5, 3.5, 3.5);
        grid.setInputCloud (cloud_source);
        grid.filter (*cloud_source_filtered);
        grid2.setLeafSize (3.5, 3.5, 3.5);
        grid2.setInputCloud (cloud_target);
        grid2.filter (*cloud_target_filtered);
        FILE_LOG(logINFO) << " post size :" << cloud_source_filtered->size() << " ; " << cloud_target_filtered->size();


        registration.setInputSource(cloud_source_filtered);
        registration.setInputTarget(cloud_target_filtered);

        //
        registration.setTransformationEpsilon (1e-8);
        registration.setMaxCorrespondenceDistance (0.5);
//        registration.setMaximumIterations (30);
        registration.setRANSACIterations(2000);
        registration.setMaximumIterations(1000);
        registration.setEuclideanFitnessEpsilon(1e-5); //1);

        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        registration.align(*cloud_source_to_target_downsampled);
        if (registration.hasConverged())
        {

            // Transform target back in source frame
//            pcl::transformPointCloud (*cloud_source, *cloud_source_to_target_full, registration.getFinalTransformation());
//            pcl::VoxelGrid<pcl::PointXYZRGB> grid_out;
//            FILE_LOG(logINFO) << " output cloud size pre downsampling :" << cloud_source_to_target_downsampled->size();
//            grid_out.setLeafSize (3, 3, 3);
//            grid_out.setInputCloud (cloud_source_to_target_downsampled);
//            grid_out.filter (*cloud_source_to_target_downsampled);
//            FILE_LOG(logINFO) << " output cloud size post downsampling :" << cloud_source_to_target_downsampled->size();

            FILE_LOG(logINFO) << "ICP converged." << "The score is " << registration.getFitnessScore();
//            std::cout << "Transformation matrix:" << std::endl;
            std::cout << registration.getFinalTransformation() << std::endl;
            transformMatrix = registration.getFinalTransformation();
        }
        else FILE_LOG(logINFO) << "ICP did not converge.";


        return transformMatrix;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr naiveRegistrationCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target){


        //    // ICP object.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_to_target_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::IterativeClosestPointNonLinear<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

        FILE_LOG(logINFO) << " original size :" << cloud_source->size() << " ; " << cloud_target->size();
        pcl::VoxelGrid<pcl::PointXYZRGB> grid, grid2;
        grid.setLeafSize (3, 3, 3);
        grid.setInputCloud (cloud_source);
        grid.filter (*cloud_source_filtered);
        grid2.setLeafSize (3, 3, 3);
        grid2.setInputCloud (cloud_target);
        grid2.filter (*cloud_target_filtered);
        FILE_LOG(logINFO) << " post size :" << cloud_source_filtered->size() << " ; " << cloud_target_filtered->size();


        registration.setInputSource(cloud_source_filtered);
        registration.setInputTarget(cloud_target_filtered);

        //
        registration.setTransformationEpsilon (1e-8);
        registration.setMaxCorrespondenceDistance (0.5);
        registration.setMaximumIterations (30);
//        registration.setRANSACIterations(2000);
//        registration.setMaximumIterations(1000);
        registration.setEuclideanFitnessEpsilon(1e-5); //1);

        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        registration.align(*cloud_source_to_target_downsampled);
        if (registration.hasConverged())
        {

            // Transform target back in source frame
//            pcl::transformPointCloud (*cloud_source, *cloud_source_to_target_full, registration.getFinalTransformation());
//            pcl::VoxelGrid<pcl::PointXYZRGB> grid_out;
//            FILE_LOG(logINFO) << " output cloud size pre downsampling :" << cloud_source_to_target_downsampled->size();
//            grid_out.setLeafSize (3, 3, 3);
//            grid_out.setInputCloud (cloud_source_to_target_downsampled);
//            grid_out.filter (*cloud_source_to_target_downsampled);
//            FILE_LOG(logINFO) << " output cloud size post downsampling :" << cloud_source_to_target_downsampled->size();

            FILE_LOG(logINFO) << "ICP converged." << "The score is " << registration.getFitnessScore();
//            std::cout << "Transformation matrix:" << std::endl;
            std::cout << registration.getFinalTransformation() << std::endl;
            transformMatrix = registration.getFinalTransformation();
        }
        else FILE_LOG(logINFO) << "ICP did not converge.";


        return cloud_source_to_target_downsampled;
    }





}



//    ////////////////////////////////////////////////////////////////////////////////
//    void
//    findCorrespondences (  pcl::PointCloud<pcl::PointNormal>::Ptr  &src,
//            pcl::PointCloud<pcl::PointNormal>::Ptr  &tgt,
//            pcl::Correspondences &all_correspondences)
//    {
//        //CorrespondenceEstimationNormalShooting<PointT, PointT, PointT> est;
//        //CorrespondenceEstimation<PointT, PointT> est;
//        CorrespondenceEstimationBackProjection<PointNormal, PointNormal, PointNormal> est;
//        est.setInputSource (src);
//        est.setInputTarget (tgt);
//
//        est.setSourceNormals (src);
//        est.setTargetNormals (tgt);
//        est.setKSearch (10);
//        est.determineCorrespondences (all_correspondences);
//        //est.determineReciprocalCorrespondences (all_correspondences);
//    }
//
//    ////////////////////////////////////////////////////////////////////////////////
//    void
//    rejectBadCorrespondences (  pcl::CorrespondencesPtr &all_correspondences,
//            pcl::PointCloud<pcl::PointNormal>::Ptr  &src,
//            pcl::PointCloud<pcl::PointNormal>::Ptr  &tgt,
//            pcl::Correspondences &remaining_correspondences)
//    {
//        CorrespondenceRejectorMedianDistance rej;
//        rej.setMedianFactor (8.79241104);
//        rej.setInputCorrespondences (all_correspondences);
//
//        rej.getCorrespondences (remaining_correspondences);
//        return;
//
//        CorrespondencesPtr remaining_correspondences_temp (new Correspondences);
//        rej.getCorrespondences (*remaining_correspondences_temp);
//        PCL_DEBUG ("[rejectBadCorrespondences] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());
//
//        // Reject if the angle between the normals is really off
//        CorrespondenceRejectorSurfaceNormal rej_normals;
//        rej_normals.setThreshold (acos (deg2rad (45.0)));
//        rej_normals.initializeDataContainer<PointNormal, PointNormal> ();
//        rej_normals.setInputCloud<PointNormal> (src);
//        rej_normals.setInputNormals<PointNormal, PointNormal> (src);
//        rej_normals.setInputTarget<PointNormal> (tgt);
//        rej_normals.setTargetNormals<PointNormal, PointNormal> (tgt);
//        rej_normals.setInputCorrespondences (remaining_correspondences_temp);
//        rej_normals.getCorrespondences (remaining_correspondences);
//    }
//
//    ////////////////////////////////////////////////////////////////////////////////
//    void
//    findTransformation (  pcl::PointCloud<pcl::PointNormal>::Ptr  &src,
//            pcl::PointCloud<pcl::PointNormal>::Ptr  &tgt,
//            pcl::CorrespondencesPtr &correspondences,
//            Eigen::Matrix4d &transform)
//    {
//        TransformationEstimationPointToPlaneLLS<PointNormal, PointNormal, double> trans_est;
//        trans_est.estimateRigidTransformation (*src, *tgt, *correspondences, transform);
//    }
//
//    ////////////////////////////////////////////////////////////////////////////////
//    void
//    view (  pcl::PointCloud<pcl::PointNormal>::Ptr &src,   pcl::PointCloud<pcl::PointNormal>::Ptr &tgt,   pcl::CorrespondencesPtr &correspondences)
//    {
//        if (!visualize || !vis) return;
//        PointCloudColorHandlerCustom<PointNormal> green (tgt, 0, 255, 0);
//        if (!vis->updatePointCloud<PointNormal> (src, "source"))
//        {
//            vis->addPointCloud<PointNormal> (src, "source");
//            vis->resetCameraViewpoint ("source");
//        }
//        if (!vis->updatePointCloud<PointNormal> (tgt, green, "target")) vis->addPointCloud<PointNormal> (tgt, green, "target");
//        vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
//        vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.7, "target");
//        vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source");
//        pcl::console::TicToc tt;
//        tt.tic ();
//        if (!vis->updateCorrespondences<PointNormal> (src, tgt, *correspondences, 1))
//            vis->addCorrespondences<PointNormal> (src, tgt, *correspondences, 1, "correspondences");
//        tt.toc_print ();
//        vis->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 5, "correspondences");
//        //vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
//        vis->spin ();
//    }
//
//    ////////////////////////////////////////////////////////////////////////////////
//    void
//    icp (  pcl::PointCloud<pcl::PointNormal>::Ptr &src,
//            pcl::PointCloud<pcl::PointNormal>::Ptr &tgt,
//            Eigen::Matrix4d &transform)
//    {
//
//        vis.reset (new PCLVisualizer ("Registration example"));
//
//        pcl::CorrespondencesPtr all_correspondences (new  pcl::Correspondences);
//        pcl::CorrespondencesPtr good_correspondences (new  pcl::Correspondences);
//
//        pcl::PointCloud<pcl::PointNormal>::Ptr output (new  pcl::PointCloud<pcl::PointNormal>);
//        *output = *src;
//
//        Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity ());
//
//        int iterations = 0;
//        DefaultConvergenceCriteria<double> converged (iterations, transform, *good_correspondences);
//
//        // ICP loop
//        do
//        {
//            // Find correspondences
//            findCorrespondences (output, tgt, *all_correspondences);
//            PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());
//
//            if (rejection)
//            {
//                // Reject correspondences
//                rejectBadCorrespondences (all_correspondences, output, tgt, *good_correspondences);
//                PCL_DEBUG ("Number of correspondences remaining after rejection: %d\n", good_correspondences->size ());
//            }
//            else
//                *good_correspondences = *all_correspondences;
//
//            // Find transformation
//            findTransformation (output, tgt, good_correspondences, transform);
//
//            // Obtain the final transformation
//            final_transform = transform * final_transform;
//
//            // Transform the data
//            transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());
//
//            // Check if convergence has been reached
//            ++iterations;
//
//            // Visualize the results
//            view (output, tgt, good_correspondences);
//        }
//        while (!converged);
//        transform = final_transform;
//    }
//
//
//}
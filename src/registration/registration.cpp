
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
                    *batch_cloud_sum += *(stereo_registration::naiveRegistration(clouds_to_register[j], clouds_to_register[start]));

                }
            }

            output_batches_of_registered_clouds.push_back(batch_cloud_sum);
            FILE_LOG(logINFO) << "BATCH point size =  "<< batch_cloud_sum->size();


        }

        FILE_LOG(logINFO) << "We have a total of :"<< output_batches_of_registered_clouds.size() << " batches";

        return output_batches_of_registered_clouds;

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  naiveRegistration( pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target){


        //    // ICP object.
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_to_target(new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;
        registration.setInputSource(cloud_source);
        registration.setInputTarget(cloud_target);

        //
        registration.setTransformationEpsilon (1e-8);
        registration.setMaxCorrespondenceDistance (0.05);
        registration.setMaximumIterations (30);
        registration.setEuclideanFitnessEpsilon(1);

        registration.align(*cloud_source_to_target);
        if (registration.hasConverged())
        {
            FILE_LOG(logINFO) << "ICP converged.";
//                    << "The score is " << registration.getFitnessScore() << std::endl;
//            std::cout << "Transformation matrix:" << std::endl;
            std::cout << registration.getFinalTransformation() << std::endl;
        }
        else FILE_LOG(logINFO) << "ICP did not converge.";


        return cloud_source_to_target;
    }


    void icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_sr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_tg,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, const bool downsample, Eigen::Matrix4f &final_transform){
//
        ///downsampling
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> grid, grid2;
//
        FILE_LOG(logINFO) << " original size :" << cloud_sr->size() << " ; " << cloud_tg->size();

        if (downsample) {
            grid.setLeafSize (1, 1, 1);
            grid.setInputCloud (cloud_sr);
            grid.filter (*src);
//
            grid2.setLeafSize (2, 2, 2);
            grid2.setInputCloud (cloud_tg);
            grid2.filter (*tgt);
        }
        else {
            src = cloud_sr;
            tgt = cloud_tg;
        }

        FILE_LOG(logINFO) << " post size :" << src->size() << " ; " << tgt->size();

//    // Compute surface normals and curvature
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
//
//    //run iterativatly icp
//
        pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;

//    //PARAMETRI DA SETTARE
        reg.setTransformationEpsilon (1e-8);
        // Set the maximum distance between two correspondences (src<->tgt) to 10cm
        // Note: adjust this based on the size of your datasets
        reg.setMaxCorrespondenceDistance (0.02);

        reg.setInputSource(points_with_normals_src);
        reg.setInputTarget(points_with_normals_tgt);


        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
        pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;

        reg.setMaximumIterations (2); //era 2
//
        //i andava fino a 30
        for (int i = 0; i < 2; ++i) {
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
//
        // Get the transformation from target to source
        targetToSource = Ti.inverse();

        //
        // Transform target back in source frame
        pcl::transformPointCloud (*cloud_tg, *cloud_out, targetToSource);

        //add the source to the transformed target
        *cloud_out += *cloud_sr;

        final_transform = targetToSource;
//
//        reg.align(*cloud_out);
//        std::cout << "ICP has converged = " << reg.hasConverged() <<endl;
//        // Save the transformed cloud
//        pcl::io::savePCDFileASCII ("cloud_after_icp.pcd", *cloud_out);
//        // Save the transformation
//        std::ofstream out2("transform_icp.txt");
//        Eigen::Affine3f Tr_icp;Tr_icp = reg.getFinalTransformation ();
//        cout<<"ICP transformation: "<<endl;
//        for(int i=0; i<4; i++){
//            for(int j=0; j<4; j++){
//                out2<<" "<<Tr_icp(i,j);
//                cout<<Tr_icp(i,j)<<"\t";
//            }
//            cout<<endl;
//            out2<<endl;
//        }
//        out2.close();


    }

//
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registerClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds) {
//
        unsigned int cloud_number = (unsigned int) clouds.size();
        Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
//
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 1; i < cloud_number; i++) {

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

            source = clouds[i - 1];
            target = clouds[i];

            icp(source, target, temp, true, pairTransform);
            //transform current pair into the global transform
            pcl::transformPointCloud(*temp, *result, GlobalTransform);
            //update the global transform
            GlobalTransform = GlobalTransform * pairTransform;

            //save aligned pair, transformed into the first cloud's frame

        }

//        std::stringstream ss;
//        ss << "registrazione.pcd";
//        pcl::io::savePCDFile(ss.str(), *result, true);

//        stereo::viewPointCloud(result);

        return result;

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
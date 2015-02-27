#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>


// local include
#include "registration.hpp"
#include "../stereo_viewer/viewer.hpp"


// Include logging facilities
#include "../logger/log.h"

class VoxelGrid;

#include <pcl/filters/voxel_grid.h>


namespace stereo_registration {



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
        stereo::viewPointCloud(result);

    }


    void icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_sr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_tg,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, const bool downsample, Eigen::Matrix4f &final_transform){

    ///downsampling
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;

    if (downsample) {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_sr);
        grid.filter (*src);

        grid.setInputCloud (cloud_tg);
        grid.filter (*tgt);
    }
    else {
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
    for (int i = 0; i < 5; ++i) {
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

}
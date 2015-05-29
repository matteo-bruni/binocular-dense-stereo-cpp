#ifndef BINOCULAR_DENSE_STEREO_VIEWER_HPP
#define BINOCULAR_DENSE_STEREO_VIEWER_HPP

#include "../includes.h"

namespace binocular_dense_stereo {

    using pcl::visualization::PointCloudColorHandlerGenericField;
    using pcl::visualization::PointCloudColorHandlerCustom;


    void viewPointCloud(PointCloud::Ptr point_cloud_ptr,  std::string title = "NoTitle");
    void viewPointCloudRGB(PointCloudRGB::Ptr point_cloud_ptr,  std::string title = "NoTitle");

    void viewDoublePointCloud(PointCloud::Ptr point_cloud_ptr, PointCloud::Ptr point_cloud_ptr2,  std::string title = "NoTitle");
    void viewDoublePointCloudRGB(PointCloudRGB::Ptr point_cloud_ptr, PointCloudRGB::Ptr point_cloud_ptr2,  std::string title = "NoTitle");


    boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (PointCloud::ConstPtr cloud, std::string title = "NoTitle");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizerRGB (PointCloudRGB::ConstPtr cloud, std::string title = "NoTitle");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> createDoubleVisualizer (
            PointCloud::ConstPtr cloud, PointCloud::ConstPtr cloud2, std::string title = "NoTitle");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> createDoubleVisualizerRGB (
            PointCloudRGB::ConstPtr cloud, PointCloudRGB::ConstPtr cloud2, std::string title = "NoTitle");


    pcl::PolygonMesh visualizerGetCameraMesh(const Eigen::Matrix3f& R, const Eigen::Vector3f& t, float r, float g, float b,
            Eigen::Vector3f& vforward, Eigen::Vector3f& rgb, double s = 0.01
    );
}

#endif
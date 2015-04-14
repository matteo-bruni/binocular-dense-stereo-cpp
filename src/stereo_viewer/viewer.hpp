#include <boost/make_shared.hpp>
#include <pcl/visualization/pcl_visualizer.h>



namespace stereo {

    using pcl::visualization::PointCloudColorHandlerGenericField;
    using pcl::visualization::PointCloudColorHandlerCustom;


    void viewPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr,  std::string title = "NoTitle");

    void viewDoublePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr2);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> createVisualizer (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::string title = "NoTitle");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> createDoubleVisualizer (
            pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud2);


    pcl::PolygonMesh visualizerGetCameraMesh(const Eigen::Matrix3f& R, const Eigen::Vector3f& t, float r, float g, float b,
            Eigen::Vector3f& vforward, Eigen::Vector3f& rgb, double s = 0.01
    );
}
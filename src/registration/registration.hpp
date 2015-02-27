#include <pcl/point_types.h>


namespace stereo_registration {



    void registerClouds( std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds);

    void icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_sr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_tg,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, const bool downsample, Eigen::Matrix4f &final_transform);


}
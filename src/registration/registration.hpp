#include <pcl/point_types.h>


namespace stereo_registration {

    struct CloudAlignment {
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
        //std::string frame_name;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedCloud;
    };


    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> register_clouds_in_batches(
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_register, int batch_size);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr register_incremental_clouds(
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_register);



    CloudAlignment registerSourceToTarget(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target);


}
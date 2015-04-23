#include <pcl/point_types.h>


namespace stereo_registration {

    struct CloudAlignment {
        //std::string frame_name;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();
    };

    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> register_clouds_in_batches(
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_register, int batch_size);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr register_incremental_clouds(
            std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_to_register);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr registerClouds( std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds);


    void icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_sr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_tg,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, const bool downsample, Eigen::Matrix4f &final_transform);

    Eigen::Matrix4f  naiveRegistrationTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  naiveRegistrationCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source,
                                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target);

    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> iterativeNaiveRegistration ( std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);
//
//    void
//            findCorrespondences ( pcl::PointCloud<pcl::PointNormal>::Ptr &src,
//             pcl::PointCloud<pcl::PointNormal>::Ptr &tgt,
//            pcl::Correspondences &all_correspondences);
//
//    void
//            rejectBadCorrespondences (  pcl::CorrespondencesPtr &all_correspondences,
//            pcl::PointCloud<pcl::PointNormal>::Ptr &src,
//            pcl::PointCloud<pcl::PointNormal>::Ptr &tgt,
//            pcl::Correspondences &remaining_correspondences);
//
//    void
//            findTransformation ( pcl::PointCloud<pcl::PointNormal>::Ptr &src,
//            pcl::PointCloud<pcl::PointNormal>::Ptr &tgt,
//            pcl::CorrespondencesPtr &correspondences,
//            Eigen::Matrix4d &transform);
//
//    void
//            view ( pcl::PointCloud<pcl::PointNormal>::Ptr &src,  pcl::PointCloud<pcl::PointNormal>::Ptr &tgt,   pcl::CorrespondencesPtr &correspondences);
//
//    void
//            icp ( pcl::PointCloud<pcl::PointNormal>::Ptr &src,
//            pcl::PointCloud<pcl::PointNormal>::Ptr &tgt,
//            Eigen::Matrix4d &transform);
//
//
}
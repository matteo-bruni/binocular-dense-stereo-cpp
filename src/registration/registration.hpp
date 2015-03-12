#include <pcl/point_types.h>


namespace stereo_registration {

//
////
    void registerClouds( std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& clouds);
////
    void icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_sr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_tg,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out, const bool downsample, Eigen::Matrix4f &final_transform);
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
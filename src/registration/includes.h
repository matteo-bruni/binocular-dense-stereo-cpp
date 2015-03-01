#pragma once

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZI PointI;
typedef pcl::PointXYZRGBNormal PointCN;
typedef pcl::Normal PointNormal;
typedef pcl::PointCloud<PointXYZ> PointCloudT;
typedef pcl::PointCloud<PointNormal> PointCloudN;
typedef pcl::PointCloud<PointC> PointCloudC;

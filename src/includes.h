#ifndef BINOCULAR_DENSE_STEREO_INCLUDES_H
#define BINOCULAR_DENSE_STEREO_INCLUDES_H

// library include
#include <boost/make_shared.hpp>

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

// Eigen
#include <Eigen/Core>

// opencv
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/types_c.h>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// std
#include <iostream>
#include <limits>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <time.h>


// local include
#include "types/custom_types.hpp"

#endif //BINOCULAR_DENSE_STEREO_INCLUDES_H

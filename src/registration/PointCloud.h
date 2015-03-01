#pragma once

#include "includes.h"
#include "constants.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/ply_io.h>

class PointCloud {
public:
	std::vector<PointCloudC::Ptr> pyramidal_downsampling;
	PointCloudC::Ptr pcl_cloud;
	Eigen::Matrix4f pose_estimation;

	PointCloud(PointCloud* other_cloud = NULL);
	PointCloud(std::string file_name);
	~PointCloud();
	void allocate();
	void deallocate();
	void reset();
	void loadCloud(std::string file_name);
	void doPyramidalDownsampling();
	void applyObjectToWorldTransformation();
};

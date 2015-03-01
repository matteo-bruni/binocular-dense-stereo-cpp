#pragma once

#include "includes.h"
#include "constants.h"
#include "PointCloud.h"
#include <pcl/surface/concave_hull.h>
#include <fstream>
using namespace std;
class PointCloudPairRegistration {
public:

	PointCloudPairRegistration(string scene_str, int pair_id, PointCloud* cloud_1, PointCloud* cloud_2);
	~PointCloudPairRegistration();
	bool file_exists(const std::string& name);
	void registration();
	void doTransformation();
	void find3DBB();
	void saveRegistration();
	void loadRegistration();

	PointCloud* point_cloud_1;
	PointCloud* point_cloud_2;
	PointCloud* point_cloud_aligned;
	PointCloud* bb_cloud;


	string scene_dataset_path;
	float final_score;
	bool registered;
	bool transformed;
	bool foundBB;
	int pair_id;
	Eigen::Matrix4f final_transformation;
	Eigen::Matrix4f final_whole_transformation;

};

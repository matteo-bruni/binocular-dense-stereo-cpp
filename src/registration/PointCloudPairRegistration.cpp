#include "PointCloudPairRegistration.h"
using namespace std;
bool PointCloudPairRegistration::file_exists(const std::string& name)
{
	ifstream f(name.c_str());
	if (f.good()) {
		f.close();
		return true;
	} else {
		f.close();
		return false;
	}
}
PointCloudPairRegistration::PointCloudPairRegistration(string scene_dataset_str, int pair_id, PointCloud* cloud_1, PointCloud* cloud_2) {
	scene_dataset_path = scene_dataset_str;
	point_cloud_1 = cloud_1;
	point_cloud_2 = cloud_2;
	point_cloud_aligned = new PointCloud(cloud_2);
	bb_cloud = new PointCloud();
	final_transformation = Eigen::Matrix4f::Identity();
	final_whole_transformation = Eigen::Matrix4f::Identity();
	registered = false;
	transformed = false;
	foundBB = false;
	this->pair_id = pair_id;
}

PointCloudPairRegistration::~PointCloudPairRegistration() {
	if(point_cloud_aligned != NULL)
		delete point_cloud_aligned;
	if(bb_cloud != NULL)
		delete bb_cloud;
}

void PointCloudPairRegistration::registration() {
	stringstream str;
	str << scene_dataset_path << "/pair-" << pair_id << ".pair";
	if(file_exists(str.str())){
		loadRegistration();
		cout << "Registration file found...skipping" << endl;
	} else {
		if(registered){
			cout << "Already registered...skipping" << endl;
		} else {
			PointCloudC::Ptr object_1(new PointCloudC);
			PointCloudC::Ptr object_2(new PointCloudC);


			final_transformation = Eigen::Matrix4f::Identity();

			cout << "-------------------------" << endl;
			cout << "Starting ICP alignment..." << endl;
			for(int i = 0; i < DOWNSAMPLE_LEVELS; i++){
				if( (i < (DOWNSAMPLE_LEVELS - 1)) || (i == (DOWNSAMPLE_LEVELS - 1) && final_score > 0.0001)){
					PointCloudC::Ptr object_2_tmp (new PointCloudC);

					cout << "Level " << i << "..."	;

					// getting the point_clouds in their initial pose at a certain level
					// of sampling
					pcl::copyPointCloud(*(point_cloud_1->pyramidal_downsampling.at(i)), *object_1);
					pcl::copyPointCloud(*(point_cloud_2->pyramidal_downsampling.at(i)), *object_2);
					pcl::IterativeClosestPoint<PointC, PointC> icp;

					icp.setRANSACIterations(2000);
					icp.setMaximumIterations(100);
					icp.setInputTarget(object_1);
					icp.setInputCloud(object_2);
					icp.align(*object_2_tmp, final_transformation);
					object_2_tmp.reset();

					Eigen::Matrix4f transformation_icp = icp.getFinalTransformation();
					cout << " score: " << icp.getFitnessScore() << endl;
					final_score = icp.getFitnessScore();
					final_transformation = transformation_icp;
				}
			}
			saveRegistration();
			registered = true;
			object_1.reset();
			object_2.reset();
		}
	}
}

void PointCloudPairRegistration::doTransformation(){
	if(transformed) {
		cout << "Already transformed...skipping" << endl;
	} else {
		PointCloudC cloud_tmp;
		pcl::copyPointCloud(*(point_cloud_aligned->pcl_cloud), cloud_tmp);
		pcl::transformPointCloud(cloud_tmp, *(point_cloud_aligned->pcl_cloud), final_transformation);
		transformed = true;
	}
}

void PointCloudPairRegistration::find3DBB(){

	bb_cloud->pcl_cloud.reset(new PointCloudC);
	PointCloudC merged_tmp;
	pcl::copyPointCloud((*(point_cloud_1->pcl_cloud))+(*(point_cloud_aligned->pcl_cloud)), merged_tmp);
	PointCloudT::Ptr merged(new PointCloudT);
	pcl::copyPointCloud(merged_tmp, *merged);
	PointCloudT output;
	pcl::ConvexHull<PointXYZ> bb;
	pcl::search::KdTree<PointXYZ>::Ptr kdtree(new pcl::search::KdTree<PointXYZ>());
	bb.setDimension(3);
	bb.setComputeAreaVolume(true);
	bb.setSearchMethod(kdtree);
	bb.setInputCloud(merged);
	bb.reconstruct(output);
	pcl::copyPointCloud(output, *(bb_cloud->pcl_cloud));
	kdtree.reset();
	merged.reset();
	foundBB = true;
}

void PointCloudPairRegistration::saveRegistration(){
	stringstream str;
	str << scene_dataset_path << "/pair-" << pair_id << ".pair";
	ofstream os(str.str().c_str());
	Eigen::Matrix4f trans = final_transformation;
	float score = final_score;
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			os << trans(i,j) << endl;
		}
	}
	os << score;
	os.close();
}

void PointCloudPairRegistration::loadRegistration(){
	stringstream str;
	str << scene_dataset_path << "/pair-" << pair_id << ".pair";
	ifstream is(str.str().c_str());
	string line;

	Eigen::Matrix4f trans = final_transformation;
	float score = 0.0f;
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){			
			std::getline(is, line);
			std::istringstream iss(line);
			iss >> trans(i,j);
		}
	}
	std::getline(is, line);
	std::istringstream iss(line);
	iss >> score;
	final_transformation = trans;
	final_score = score;
}

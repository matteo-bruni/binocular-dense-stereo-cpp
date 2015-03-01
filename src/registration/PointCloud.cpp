
#include "PointCloud.h"
#include "includes.h"

using namespace std;

PointCloud::PointCloud(PointCloud* other_cloud){
	allocate();
	if(other_cloud != NULL){
		pcl::copyPointCloud(*(other_cloud->pcl_cloud), *pcl_cloud);
	}
}

PointCloud::PointCloud(std::string file_name){
	PointCloud(NULL);
	loadCloud(file_name);
}

PointCloud::~PointCloud(){
	deallocate();
}

void PointCloud::allocate() {
	pcl_cloud.reset(new PointCloudC);
}
void PointCloud::deallocate() {
	pcl_cloud.reset();
	for(int i = 0; i < pyramidal_downsampling.size(); i++){
		if(pyramidal_downsampling[i] != NULL)
			pyramidal_downsampling[i].reset();
	}

}

void PointCloud::reset(){
	deallocate();
	allocate();
}

void PointCloud::loadCloud(std::string file_name) {
	cout << "Loading '" << file_name << "'...";
	pcl::io::loadPLYFile<PointC>(file_name, *pcl_cloud); 
	for (int i = 0; i < pcl_cloud->size(); i++) {
		PointC pcl_point = pcl_cloud->points.at(i);
		pcl_point.x /= 1000.0;
		pcl_point.y /= 1000.0;
		pcl_point.z /= 1000.0;
		pcl_point.z += 1.0;
		pcl_cloud->points[i] = pcl_point;
	}
	PointCloudC cloud_filtered;
	pcl::StatisticalOutlierRemoval<PointC> sor;
	sor.setInputCloud (pcl_cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (cloud_filtered);
	pcl::copyPointCloud(cloud_filtered, *pcl_cloud);
	doPyramidalDownsampling();
	cout << "OK (" << pcl_cloud->size() << " points)" << endl;
}

void PointCloud::doPyramidalDownsampling() {
	// Downsampling
	for(int i = 0; i < DOWNSAMPLE_LEVELS; i++){
		pcl::VoxelGrid<PointC> grid;
		PointCloudC::Ptr cloud_downsampled(new PointCloudC);
		const float leaf = LEAF_SIZE * (i+1);
		grid.setLeafSize (leaf, leaf, leaf);
		grid.setInputCloud (pcl_cloud);
		grid.filter (*cloud_downsampled);
		pyramidal_downsampling.push_back(cloud_downsampled);
	}
	reverse(pyramidal_downsampling.begin(),pyramidal_downsampling.end());
}

void PointCloud::applyObjectToWorldTransformation() {
	pcl::transformPointCloud(*pcl_cloud,*pcl_cloud,pose_estimation.inverse().eval());
	// buildMeshFromPC();
}
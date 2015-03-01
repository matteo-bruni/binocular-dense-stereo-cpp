
#pragma once

#include <pcl/common/common_headers.h>
#include "includes.h"
#include "PointCloud.h"
#include "PointCloudPairRegistration.h"

void exit();

void setupDataset(string tmp_scene_datased_path, float tmp_true_volume);
// void testCurrentDataset();
// void testAllDataset();

void loadScene(int capture_id);
void registration(int pair_id);
void loadRegistration(int pair_id);
void loadRegistrationAll();
void reconstruction(int pair_id);
void registrationAll();
void reconstructionAll();

void findConvexHull(int pair_id);
void findTotalConvexHull();
void computeConvexHullPCATransformation();

void find3DBB(int pair_id);
void findTotal3DBB();
void showTotal3DBB();

void writeResults();

int getNumViews();

void buildStructures();
void buildPointCloudStructures();
void buildPCPairStructures();


void main_registration(std::vector<PointCloudC::Ptr> clouds);

// dataset structure
vector<pair< pair<string,string> ,float> > dataset_v;

// Structures variables
vector<PointCloud*> point_cloud_v;
vector<PointCloudPairRegistration*> pc_pair;

PointCloud point_cloud_merged;
PointCloud point_cloud_convex_hull;
PointCloud point_cloud_bb;
pcl::PolygonMesh polygon_mesh;

Eigen::Matrix4f pose_estimation;

int current_pair;

float true_volume;
float est_convex_hull_volume;
float est_bb_volume;

int num_scenes;
string scene_dataset_path;

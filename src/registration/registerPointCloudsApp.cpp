#include "registerPointCloudsApp.h"
#include <pcl/ros/conversions.h>
#include <pcl/common/common_headers.h>
#include <dirent.h>


using namespace std;


	int getNumViews() {
		DIR *dp;
		std::string extension = std::string(".ply");
		struct dirent *dirp;
		if ((dp = opendir(scene_dataset_path.c_str())) == NULL) {
			std::cout << "Error(" << errno << ") opening " << scene_dataset_path << std::endl;
		}
		int file_counter = 0;
		while ((dirp = readdir(dp)) != NULL) {
			std::string file_name = std::string(dirp->d_name);
			if (file_name.substr(std::max(0, (int) (file_name.size() - 4))) == extension) {
				if (file_name.substr(0, 7) == std::string("capture"))
					file_counter++;
			}
		}
		closedir(dp);
		return file_counter;
	}


	void buildStructures() {
		buildPointCloudStructures();
	}

	void buildPointCloudStructures() {
		for (int i = 0; i < num_scenes; i++) {
			//populate PointCloud objects
			PointCloud *tmp_point_cloud = new PointCloud();
			point_cloud_v.push_back(tmp_point_cloud);
		}
	}

	void buildPCPairStructures() {
		for (int i = 1; i < num_scenes; i++) {
			//populate PointCloud objects
			PointCloudPairRegistration *tmp_pair = new PointCloudPairRegistration(scene_dataset_path, i - 1, point_cloud_v.at(i - 1), point_cloud_v.at(i));
			pc_pair.push_back(tmp_pair);
		}
	}

	void loadScene(std::vector<PointCloudC::Ptr>  clouds) {

		for (int i = 0; i < clouds.size(); i++) {
			*point_cloud_v.at(i) = clouds[i];
		}

	}

	void registration(int pair_id) {
		pc_pair.at(pair_id)->registration();
	}

	void loadRegistration(int pair_id) {
		pc_pair.at(pair_id)->loadRegistration();
	}

	void loadRegistrationAll() {
		for (int i = 0; i < pc_pair.size(); i++) {
			loadRegistration(i);
		}
	}

	void registrationAll() {
		for (int i = 0; i < pc_pair.size(); i++) {
			registration(i);
		}
	}

	void reconstruction(int pair_id) {
		pc_pair.at(pair_id)->doTransformation();
	}

	void reconstructionAll() {
		PointCloudC::Ptr final_cloud(new PointCloudC);
		for (int i = 1; i < point_cloud_v.size(); i++) {
			reconstruction(i - 1);
			PointCloudC::Ptr tmp_cloud(new PointCloudC);
			pcl::copyPointCloud(*final_cloud + *(point_cloud_v[i - 1]->pcl_cloud), *tmp_cloud);
			pcl::transformPointCloud(*tmp_cloud, *final_cloud, pc_pair.at(i - 1)->final_transformation.inverse().eval());
		}

		pcl::VoxelGrid<PointC> grid;
		PointCloudC cloud_downsampled;
		float leaf = 0.001;
		grid.setLeafSize(leaf, leaf, leaf);
		grid.setInputCloud(final_cloud->makeShared());
		grid.filter(cloud_downsampled);
		// FIXME: can we merge these two lines?
		pcl::copyPointCloud(cloud_downsampled, *final_cloud);
		pcl::copyPointCloud(*final_cloud, *(point_cloud_merged.pcl_cloud));
		findTotalConvexHull();
	}

	void findTotalConvexHull() {
		pcl::ConvexHull<PointC> bb;
		pcl::search::KdTree<PointC>::Ptr kdtree(new pcl::search::KdTree<PointC>());
		bb.setDimension(3);
		bb.setComputeAreaVolume(true);
		bb.setSearchMethod(kdtree);
		bb.setInputCloud(point_cloud_merged.pcl_cloud);
		bb.reconstruct(*(point_cloud_convex_hull.pcl_cloud), polygon_mesh.polygons);
		pcl::toROSMsg(*(point_cloud_convex_hull.pcl_cloud), polygon_mesh.cloud);
		computeConvexHullPCATransformation();
	}

	void computeConvexHullPCATransformation() {
		// starting PCA
		pcl::PCA<PointC> pca;
		pcl::PointCloud<PointC> proj;
		pca.setInputCloud(point_cloud_convex_hull.pcl_cloud->makeShared());
		pca.project(*(point_cloud_convex_hull.pcl_cloud), proj);

		// getting transformation matrix
		Eigen::Matrix3f rot_mat = pca.getEigenVectors();
		Eigen::Vector3f cl_translation = pca.getMean().head(3);
		Eigen::Matrix4f pca_transformation = Eigen::Matrix4f::Identity();
		pca_transformation.col(0) << rot_mat.col(0), 0;
		pca_transformation.col(1) << -rot_mat.col(1), 0;
		pca_transformation.col(2) << rot_mat.col(2), 0;
		pca_transformation.col(3) << cl_translation, 1;

		// setting global pose_estimation
		pose_estimation = pca_transformation;

		// transforming a copy of the convex hull to be aligned to the camera axis
		PointCloudC tmp_cloud;
		pcl::copyPointCloud(*(point_cloud_convex_hull.pcl_cloud), tmp_cloud);
		pcl::transformPointCloud(tmp_cloud, tmp_cloud, pca_transformation.inverse().eval());

		// building the 8 vertices of the 3D bounding box
		PointC min_point;
		PointC max_point;
		pcl::getMinMax3D(tmp_cloud, min_point, max_point);

		PointC pcl_point_1(255, 0, 0);
		pcl_point_1.x = min_point.x;
		pcl_point_1.y = min_point.y;
		pcl_point_1.z = min_point.z;
		point_cloud_bb.pcl_cloud->push_back(pcl_point_1);

		PointC pcl_point_2(255, 0, 0);
		pcl_point_2.x = max_point.x;
		pcl_point_2.y = min_point.y;
		pcl_point_2.z = min_point.z;
		point_cloud_bb.pcl_cloud->push_back(pcl_point_2);

		PointC pcl_point_3(255, 0, 0);
		pcl_point_3.x = max_point.x;
		pcl_point_3.y = max_point.y;
		pcl_point_3.z = min_point.z;
		point_cloud_bb.pcl_cloud->push_back(pcl_point_3);

		PointC pcl_point_4(255, 0, 0);
		pcl_point_4.x = min_point.x;
		pcl_point_4.y = max_point.y;
		pcl_point_4.z = min_point.z;
		point_cloud_bb.pcl_cloud->push_back(pcl_point_4);

		PointC pcl_point_5(255, 0, 0);
		pcl_point_5.x = min_point.x;
		pcl_point_5.y = max_point.y;
		pcl_point_5.z = max_point.z;
		point_cloud_bb.pcl_cloud->push_back(pcl_point_5);

		PointC pcl_point_6(255, 0, 0);
		pcl_point_6.x = min_point.x;
		pcl_point_6.y = min_point.y;
		pcl_point_6.z = max_point.z;
		point_cloud_bb.pcl_cloud->push_back(pcl_point_6);

		PointC pcl_point_7(255, 0, 0);
		pcl_point_7.x = max_point.x;
		pcl_point_7.y = min_point.y;
		pcl_point_7.z = max_point.z;
		point_cloud_bb.pcl_cloud->push_back(pcl_point_7);

		PointC pcl_point_8(255, 0, 0);
		pcl_point_8.x = max_point.x;
		pcl_point_8.y = max_point.y;
		pcl_point_8.z = max_point.z;
		point_cloud_bb.pcl_cloud->push_back(pcl_point_8);


		// displaying estimated 3D bounding box volume
		cout << max_point.x - min_point.x << endl;
		cout << max_point.y - min_point.y << endl;
		cout << max_point.z - min_point.z << endl;

		est_bb_volume = (max_point.x - min_point.x) * (max_point.y - min_point.y) * (max_point.z - min_point.z);
		stringstream str;
		str << setprecision(2) << est_bb_volume << " (" << ((est_bb_volume > true_volume) ? "+" : "") << (est_bb_volume - true_volume) / true_volume << "%)";

		pcl::transformPointCloud(*(point_cloud_bb.pcl_cloud), *(point_cloud_bb.pcl_cloud), pose_estimation);

		// TODO: this is the indexes pattern to draw the 3D bounding box
		// // connecting vertices to build edges of the 3D bounding box
		// point_cloud_bb.mesh->addIndex(0);
		// point_cloud_bb.mesh->addIndex(1);
		// point_cloud_bb.mesh->addIndex(1);
		// point_cloud_bb.mesh->addIndex(2);
		// point_cloud_bb.mesh->addIndex(2);
		// point_cloud_bb.mesh->addIndex(3);
		// point_cloud_bb.mesh->addIndex(3);
		// point_cloud_bb.mesh->addIndex(0);
		// point_cloud_bb.mesh->addIndex(3);
		// point_cloud_bb.mesh->addIndex(4);
		// point_cloud_bb.mesh->addIndex(4);
		// point_cloud_bb.mesh->addIndex(5);
		// point_cloud_bb.mesh->addIndex(5);
		// point_cloud_bb.mesh->addIndex(0);
		// point_cloud_bb.mesh->addIndex(5);
		// point_cloud_bb.mesh->addIndex(6);
		// point_cloud_bb.mesh->addIndex(6);
		// point_cloud_bb.mesh->addIndex(1);
		// point_cloud_bb.mesh->addIndex(6);
		// point_cloud_bb.mesh->addIndex(7);
		// point_cloud_bb.mesh->addIndex(7);
		// point_cloud_bb.mesh->addIndex(2);
		// point_cloud_bb.mesh->addIndex(7);
		// point_cloud_bb.mesh->addIndex(4);
	}

	void writeResults() {
		stringstream str;
		str << "results_" << scene_dataset_path << ".txt";
		ofstream os(str.str().c_str());

		os << true_volume << endl;
		os << est_convex_hull_volume << endl;
		os << est_bb_volume;
		for (int i = 0; i < pc_pair.size(); i++) {
			os << endl;
			os << pc_pair.at(i)->final_score;
		}
		os.close();
	}

	void exit() {
		for (int i = 0; i < point_cloud_v.size(); i++) {
			delete point_cloud_v.at(i);
		}
		point_cloud_v.clear();
		for (int i = 0; i < pc_pair.size(); i++) {
			delete pc_pair.at(i);
		}
		point_cloud_merged.reset();
		point_cloud_convex_hull.reset();
		point_cloud_bb.reset();

		pose_estimation = Eigen::Matrix4f::Identity();
		pc_pair.clear();
	}

	void saveClouds() {
		stringstream str;
		str << scene_dataset_path << "/output-convex-hull.ply";
		pcl::io::savePLYFile(str.str(), polygon_mesh);
		str.str("");
		str.clear();
		str << scene_dataset_path << "/output-aligned-clouds.ply";
		pcl::io::savePLYFile(str.str(), *(point_cloud_merged.pcl_cloud));
	}

	void setupDataset(float tmp_true_volume, std::vector<PointCloudC::Ptr>  clouds) {


		true_volume = tmp_true_volume;

		num_scenes = getNumViews();

		// populate structures based on num_scenes
		buildStructures();


		loadScene(clouds);

		buildPCPairStructures();
		current_pair = 0;
	}

	void main_registration(std::vector<PointCloudC::Ptr> clouds) {

		true_volume = 0.0;

		setupDataset(true_volume,clouds);
		registrationAll();
		reconstructionAll();
		saveClouds();


	}

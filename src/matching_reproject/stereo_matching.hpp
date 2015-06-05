#ifndef BINOCULAR_DENSE_STEREO_STEREO_MATCHING_HPP
#define BINOCULAR_DENSE_STEREO_STEREO_MATCHING_HPP

#include "../includes.h"
#include "../dataset/msm_middlebury.hpp"
#include "../dataset/tsukuba_dataset.h"
#include "../dataset/kitti_dataset.h"

#include "../graph_cuts/match.h"
#include <limits>
#include <cmath>
#include <ctime>
#include "../graph_cuts/io_png.h"
#include "../graph_cuts/image.h"

namespace binocular_dense_stereo {

    using namespace cv;


    // used
    cv::datasets::FramePair rectifyImages(Mat& img1, Mat& img2, Mat& M1, Mat& D1, Mat& M2, Mat& D2, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Rect &roi1, Rect &roi2, float scale);

    void createPointCloudOpenCV(Mat& img1, Mat& img2, Mat& Q, Mat& disp, Mat& recons3D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr);
    void createPointCloudOpenCVKITTI(Mat& img1, Mat& img2, Mat& Q, Mat& disp, Mat& recons3D, PointCloudRGB::Ptr &point_cloud_ptr);

    void createAllCloudsTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clouds, int first_frame, int last_frame, int step);
    void createAllCloudsKITTI(Ptr<cv::datasets::SLAM_kitti> &dataset, std::vector<PointCloudRGB::Ptr> & clouds, int first_frame, int last_frame, int step);
    void createAllClouds(Ptr<cv::datasets::Dataset> &dataset, std::vector<PointCloud::Ptr> & clouds, int first_frame, int last_frame, int step);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloudTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset, const int frame_num);
    PointCloudRGB::Ptr generatePointCloudKITTI(Ptr<cv::datasets::SLAM_kitti> &dataset, const int frame_num);


    void depthFromDisparity (cv::Mat& disparity_image, float focal, float baseline_, float min_disparity, cv::Mat& depth_image, bool gt);
    void pointcloudFromDepthImage (cv::Mat& depth_image, cv::Mat& img_left, cv::Mat& depth_intrinsics, PointCloudRGB::Ptr& output_cloud);

    // legacy
    void computeDisparity(const int img1_num, const int img2_num,  Mat& img_left, Mat& img_right, Mat& disp,int alg,Rect & roi1,Rect &roi2);
    void computeDisparityTsukuba(const int img_frame, Mat& img_left, Mat& img_right,Mat& disp);
    void display(const int img1_num, const int img2_num, Mat& img1, Mat& img2,Mat& disp);
    void storePointCloud(Mat& disp, Mat& Q,/*const char* filename,*/ Mat& recons3D);
    void createPointCloudCustom (Mat& img1, Mat& img2, Mat img_1_segm, Mat& Q, Mat& disp, Mat& recons3D, PointCloud::Ptr &point_cloud_ptr);
    PointCloudRGB::Ptr generatePointCloud(Ptr<cv::datasets::MSM_middlebury> &dataset, const int img1_num, const int img2_num, bool opencv_rec);
    void createAllClouds(Ptr<cv::datasets::MSM_middlebury> &dataset, std::vector<PointCloudRGB::Ptr> & clouds);


    bool isGray(RGBImage im);
    void convert_gray(GeneralImage& im);
    void set_fractions(Match::Parameters& params,
                       float K, float lambda1, float lambda2);
    void fix_parameters(Match& m, Match::Parameters& params,
                        float& K, float& lambda, float& lambda1, float& lambda2);
    void compute_disparity_graphcuts(const int img_frame, Mat& img_left, Mat& img_right, Mat& disp);



}

#endif
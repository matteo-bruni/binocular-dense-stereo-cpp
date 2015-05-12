#ifndef BINOCULAR_DENSE_STEREO_STEREO_MATCHING_HPP
#define BINOCULAR_DENSE_STEREO_STEREO_MATCHING_HPP

#include "../includes.h"
#include "../dataset/msm_middlebury.hpp"
#include "../dataset/tsukuba_dataset.h"


namespace binocular_dense_stereo {

    using namespace cv;


    // used
    void rectifyImages(Mat& img1, Mat& img2, Mat& M1, Mat& D1, Mat& M2, Mat& D2, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Rect &roi1, Rect &roi2, float scale);

    void createPointCloudOpenCV(Mat& img1, Mat& img2, Mat& Q, Mat& disp, Mat& recons3D, PointCloud::Ptr &point_cloud_ptr);

    void createAllCloudsTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset, std::vector<PointCloud::Ptr> & clouds, int first_frame, int last_frame, int step);
    void createAllClouds(Ptr<cv::datasets::Dataset> &dataset, std::vector<PointCloud::Ptr> & clouds, int first_frame, int last_frame, int step);

    PointCloud::Ptr generatePointCloudTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset, const int frame_num);


    // legacy
    void computeDisparity(const int img1_num, const int img2_num,  Mat& img_left, Mat& img_right, Mat& disp,int alg,Rect & roi1,Rect &roi2);
    void computeDisparityTsukuba(const int img_frame, Mat& img_left, Mat& img_right,Mat& disp,int alg,Rect & roi1,Rect &roi2);
    void display(const int img1_num, const int img2_num, Mat& img1, Mat& img2,Mat& disp);
    void storePointCloud(Mat& disp, Mat& Q,/*const char* filename,*/ Mat& recons3D);
    void createPointCloudCustom (Mat& img1, Mat& img2, Mat img_1_segm, Mat& Q, Mat& disp, Mat& recons3D, PointCloud::Ptr &point_cloud_ptr);
    PointCloud::Ptr generatePointCloud(Ptr<cv::datasets::MSM_middlebury> &dataset, const int img1_num, const int img2_num, bool opencv_rec);
    void createAllClouds(Ptr<cv::datasets::MSM_middlebury> &dataset, std::vector<PointCloud::Ptr> & clouds);


}

#endif
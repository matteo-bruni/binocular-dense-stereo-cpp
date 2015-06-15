#ifndef BINOCULAR_DENSE_STEREO_STEREO_UTILS_HPP
#define BINOCULAR_DENSE_STEREO_STEREO_UTILS_HPP

#include "../includes.h"

#include "../dataset/msm_middlebury.hpp"
#include "../dataset/tsukuba_dataset.h"
#include "../dataset/dataset.hpp"

#include "../logger/log.h"
#include "../dataset/kitti_dataset.h"


namespace binocular_dense_stereo {

    using namespace std;
    using namespace cv;

    string type2str(int type);

    string infoMatrix(Mat& M);

    void rotate(cv::Mat& src, double angle, cv::Mat& dst);

    std::tuple<cv::Mat,cv::Mat> segmentation(cv::Mat image);
    cv::Mat segmentationGrabcut(cv::Mat image);

    void rotate_clockwise(cv::Mat& src, cv::Mat& dst, bool clockwise);

    cv::Mat createPINVFromRT(cv::Mat R, cv::Mat T);

    Eigen::Matrix4d getTransformToWorldCoordinatesMiddlebury(Ptr<cv::datasets::MSM_middlebury> &dataset, const int img1_num);

    Eigen::Matrix4d getTransformToWorldCoordinatesTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset,
                                                          const int current_frame);

    Eigen::Matrix4d getTransformToWorldCoordinatesKITTI(Ptr<cv::datasets::SLAM_kitti> &dataset,
                                                          const int current_frame);

    void saveVectorCloudsToPLYRGB(std::vector<PointCloudRGB::Ptr> clouds_array, std::string title);
    std::vector<PointCloudRGB::Ptr> loadVectorCloudsFromPLYRGB(std::string path, int number_of_clouds);

    std::vector<PointCloudRGB::Ptr> loadVectorCloudsFromPCDRGB(std::string path, int number_of_clouds);
    void saveVectorCloudsToPCDRGB(std::vector<PointCloudRGB::Ptr> clouds_array, std::string title);

}

#endif
#include <string>
#include <cv.h>
#include <pcl/common/common_headers.h>

#include "../dataset/msm_middlebury.hpp"
#include "../dataset/tsukuba_dataset.h"

namespace stereo_util {

    using namespace std;
    using namespace cv;

    string type2str(int type);

    string infoMatrix(Mat& M);

    void rotate(cv::Mat& src, double angle, cv::Mat& dst);

    std::tuple<cv::Mat,cv::Mat> segmentation(cv::Mat image);
    cv::Mat segmentationGrabcut(cv::Mat image);

    void rotate_clockwise(cv::Mat& src, cv::Mat& dst, bool clockwise);

    cv::Mat createPINVFromRT(cv::Mat R, cv::Mat T);

    Eigen::Matrix4f getTransformBetweenClouds(Ptr<cv::datasets::MSM_middlebury> &dataset, const int img1_num, const int img2_num);

    Eigen::Matrix4d getTransformToWorldCoordinatesTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset,
                                                          const int current_frame);

    void saveVectorCloudsToPLY(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds_array, std::string title);

}
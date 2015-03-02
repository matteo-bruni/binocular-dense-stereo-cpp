#include <string>
#include <cv.h>
#include <pcl/common/common_headers.h>

#include "../dataset/msm_middlebury.hpp"


namespace stereo_util {

    using namespace std;
    using namespace cv;

    string type2str(int type);

    string infoMatrix(Mat& M);

    void rotate(cv::Mat& src, double angle, cv::Mat& dst);

    std::tuple<cv::Mat,cv::Mat> segmentation(cv::Mat image);

    void rotate_clockwise(cv::Mat& src, cv::Mat& dst, bool clockwise);

    Eigen::Matrix4f getTransformBetweenClouds(Ptr<cv::datasets::MSM_middlebury> &dataset, const int img1_num, const int img2_num);


}
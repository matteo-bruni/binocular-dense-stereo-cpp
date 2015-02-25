#include <string>
#include <cv.h>


namespace stereo_util {

    using namespace std;
    using namespace cv;

    string type2str(int type);
    string infoMatrix(Mat& M);
    void rotate(cv::Mat& src, double angle, cv::Mat& dst);
    std::tuple<cv::Mat,cv::Mat> segmentation(cv::Mat image);

    void rotate_clockwise(cv::Mat& src, cv::Mat& dst, bool clockwise);


}
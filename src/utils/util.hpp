#include <string>
#include <cv.h>


namespace stereo_util {

    using namespace std;
    using namespace cv;

    string type2str(int type);
    string infoMatrix(Mat& M);
    void rotate(cv::Mat& src, double angle, cv::Mat& dst);


}

#include "util.hpp"
#include <opencv2/core/types_c.h>

namespace stereo_util {

    using namespace std;

    string type2str(int type) {
        string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch (depth) {
            case CV_8U:
                r = "8U";
                break;
            case CV_8S:
                r = "8S";
                break;
            case CV_16U:
                r = "16U";
                break;
            case CV_16S:
                r = "16S";
                break;
            case CV_32S:
                r = "32S";
                break;
            case CV_32F:
                r = "32F";
                break;
            case CV_64F:
                r = "64F";
                break;
            default:
                r = "User";
                break;
        }

        r += "C";
        r += (chans + '0');

        return r;
    }

    string infoMatrix(Mat& M) {
        std::ostringstream out;
        out << "Matrix: " << stereo_util::type2str( M.type() )<< " "<< M.rows << "x" << M.cols;
        return out.str();


    }

    /**
    * Rotate an image
    */
    void rotate(cv::Mat& src, double angle, cv::Mat& dst)
    {
        int len = std::max(src.cols, src.rows);
        cv::Point2f pt(len/2., len/2.);
        cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

        cv::warpAffine(src, dst, r, cv::Size(len, len));
    }

}
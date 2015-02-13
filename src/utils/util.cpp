
#include "util.hpp"
#include <opencv2/core/types_c.h>

namespace util {

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

    void infoMatrix(Mat& M) {
        string ty2 =  util::type2str( M.type() );
        printf("Matrix: %s %dx%d \n", ty2.c_str(),  M.rows, M.cols );


    }


}
#include <cv.h>
#include <highgui.h>


namespace stereo {

    using namespace cv;

    void loadImages(const std::string &img1_filename, const std::string &img2_filename,  Mat& img1,  Mat& img2);

    void rectifyImages(Mat& img1, Mat& img2, Mat& M1, Mat& D1, Mat& M2, Mat& D2, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Rect &roi1, Rect &roi2, int scale);

    void computeDisparity(Mat& img1, Mat& img2,Mat& disp,int alg);

    void display(Mat& img1, Mat& img2,Mat& disp);

    void storePointCloud(Mat& disp, Mat& Q,const char* filename, const Mat& mat);
}
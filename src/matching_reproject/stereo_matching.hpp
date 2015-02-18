#include <cv.h>
#include <highgui.h>
#include <pcl/common/common_headers.h>

namespace stereo {

    using namespace cv;

    void loadImages(const int img1_num, const int img2_num, Mat &img1, Mat &img2);

    void rectifyImages(Mat& img1, Mat& img2, Mat& M1, Mat& D1, Mat& M2, Mat& D2, Mat& R, Mat& T, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, Rect &roi1, Rect &roi2, float scale);

    void computeDisparity(Mat& img1, Mat& img2,Mat& disp,int alg);

    void display(Mat& img1, Mat& img2,Mat& disp);

    void storePointCloud(Mat& disp, Mat& Q,/*const char* filename,*/ Mat& recons3D);

    void createPointCloud(Mat& img1, Mat& img2, Mat& Q, Mat& disp, Mat& recons3D, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr);

}
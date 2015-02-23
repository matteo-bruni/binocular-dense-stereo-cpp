
#include "util.hpp"
#include <opencv2/core/types_c.h>


#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace stereo_util {

    using namespace std;
    using namespace cv;

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
//        int len = std::max(src.cols, src.rows);
        cv::Point2f pt((src.cols/2), (src.rows/2) );
        cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

        cv::warpAffine(src, dst, r, cv::Size(src.rows, src.cols));
    }

    void rotate_clockwise(cv::Mat& src, cv::Mat& dst, bool clockwise){
        if (clockwise){
            cv::transpose(src, dst);
            cv::flip(dst, dst, 1);
        } else {
            cv::transpose(src, dst);
            cv::flip(dst, dst, 0);
        }


    }

    class WatershedSegmenter{
    private:
        cv::Mat markers;
    public:
        void setMarkers(cv::Mat& markerImage)
        {
            markerImage.convertTo(markers, CV_32S);
        }

        cv::Mat process(cv::Mat &image)
        {
            cv::watershed(image, markers);
            markers.convertTo(markers,CV_8U);
            return markers;
        }
    };

    cv::Mat segmentation(cv::Mat image)
    {
//        std::ostringstream ss;
//        ss << std::setw(2) << std::setfill('0') << img_num;
//        std::string img1_path = "../dataset/dataset_templeRing/templeR00"+ ss.str() +".png";
//        cv::Mat image = cv::imread(img1_path);


        cv::Mat binary;// = cv::imread(argv[2], 0);
        cv::cvtColor(image, binary, CV_BGR2GRAY);
        cv::threshold(binary, binary, 100, 255, THRESH_BINARY);

        cv::Mat const structure_elem = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(7, 7));
//        cv::Mat open_result;
        cv::morphologyEx(binary, binary,
                cv::MORPH_CLOSE, structure_elem);


        vector<vector<Point>> contours; // Vector for storing contour
        vector<Vec4i> hierarchy;
        findContours( binary, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        // iterate through each contour.
        for( int i = 0; i< contours.size(); i++ )
        {
            //  Find the area of contour
            double a=contourArea( contours[i],false);
            Scalar color( 255,255,255);  // color of the contour in the
            drawContours( binary, contours,i, 255, CV_FILLED,8,hierarchy);

        }

        cv::Mat result1;
        image.copyTo(result1, binary);

        return result1;

//        imshow("filtrata",result1);
//
//        cv::waitKey(0);

    }

}
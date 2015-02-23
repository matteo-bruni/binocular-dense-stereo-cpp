
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
        int len = std::max(src.cols, src.rows);
        cv::Point2f pt(len/2., len/2.);
        cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

        cv::warpAffine(src, dst, r, cv::Size(len, len));
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

    void segmentation (const int img_num)
    {
        std::ostringstream ss;
        ss << std::setw(2) << std::setfill('0') << img_num;
        std::string img1_path = "../dataset/dataset_templeRing/templeR00"+ ss.str() +".png";
        cv::Mat image = cv::imread(img1_path);
        cv::Mat binary;// = cv::imread(argv[2], 0);
        cv::cvtColor(image, binary, CV_BGR2GRAY);
        cv::threshold(binary, binary, 100, 255, THRESH_BINARY);

        imshow("originalimage", image);
        imshow("originalbinary", binary);

        // Eliminate noise and smaller objects
        cv::Mat fg;
        cv::erode(binary,fg,cv::Mat(),cv::Point(-1,-1),2);
        imshow("fg", fg);

        // Identify image pixels without objects
        cv::Mat bg;
        cv::dilate(binary,bg,cv::Mat(),cv::Point(-1,-1),3);
        cv::threshold(bg,bg,1, 128,cv::THRESH_BINARY_INV);
        imshow("bg", bg);


        // Create markers image
        cv::Mat markers(binary.size(),CV_8U,cv::Scalar(0));
        markers= fg+bg;
        imshow("markers", markers);

        // Create watershed segmentation object
        WatershedSegmenter segmenter;
        segmenter.setMarkers(markers);

        cv::Mat result = segmenter.process(image);
        result.convertTo(result,CV_8U);
        imshow("final_result", result);



        cv::Mat const structure_elem = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(7, 7));
//        cv::Mat open_result;
        cv::morphologyEx(binary, binary,
                cv::MORPH_CLOSE, structure_elem);


        cv::Mat result1;
        image.copyTo(result1, binary);


        imshow("filtrata",binary);

        cv::waitKey(0);



    }

}

#include "util.hpp"
#include <opencv2/core/types_c.h>


#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "../logger/log.h"
#include "../dataset/tsukuba_dataset.h"
#include "../dataset/dataset.hpp"


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

    std::tuple<cv::Mat,cv::Mat> segmentation(cv::Mat image)
    {
//        std::ostringstream ss;
//        ss << std::setw(2) << std::setfill('0') << img_num;
//        std::string img1_path = "../dataset/dataset_templeRing/templeR00"+ ss.str() +".png";
//        cv::Mat image = cv::imread(img1_path);


        cv::Mat binary;// = cv::imread(argv[2], 0);
        cv::cvtColor(image, binary, CV_BGR2GRAY);
        cv::threshold(binary, binary, 100, 255, THRESH_BINARY);

        cv::Mat const structure_elem = cv::getStructuringElement(
                cv::MORPH_RECT, cv::Size(11, 11));
        cv::morphologyEx(binary, binary,
                cv::MORPH_CLOSE, structure_elem);

//        imshow("filtrata1",binary);

        vector<vector<Point>> contours; // Vector for storing contour
        vector<Vec4i> hierarchy;
        cv::Mat binary_contour;
        binary.copyTo(binary_contour);
        findContours( binary_contour, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
        // iterate through each contour.
        for( int i = 0; i< contours.size(); i++ )
        {
            //  Find the area of contour
            double a = contourArea( contours[i],false);
            if (a < 5000) {
                Scalar color(255, 255, 255);  // color of the contour in the
                drawContours(binary, contours, i, 255, CV_FILLED, 8, hierarchy);
            }
        }

        cv::Mat result1;
        image.copyTo(result1, binary);
//        imshow("filtrata",binary);
//        cv::waitKey(0);

//        imshow("filtrata-c",binary_contour);



        return std::make_tuple(result1, binary);

//

    }

    cv::Mat segmentationGrabcut(cv::Mat image){

//    if(! image.data ) // Check for invalid input
//    {
//        cout <<  "Could not open or find the image" << std::endl ;
//        return -1;
//    }

        // define bounding rectangle
        int border = 20;
        int border2 = border + border;
        cv::Rect rectangle(border,border,image.cols-border2,image.rows-border2);

        cv::Mat result; // segmentation result (4 possible values)
        cv::Mat bgModel,fgModel; // the models (internally used)

        // GrabCut segmentation
        cv::grabCut(image,    // input image
                result,   // segmentation result
                rectangle,// rectangle containing foreground
                bgModel,fgModel, // models
                10,        // number of iterations
                cv::GC_INIT_WITH_RECT); // use rectangle
        // Get the pixels marked as likely foreground
        cv::compare(result,cv::GC_PR_FGD,result,cv::CMP_EQ);
        // Generate output image
        cv::Mat foreground(image.size(),CV_8UC3,cv::Scalar(0,0,0));
        image.copyTo(foreground,result); // bg pixels not copied

        // draw rectangle on original image
//        cv::rectangle(image, rectangle, cv::Scalar(255,255,255),1);
//        cv::namedWindow("Image");
//        cv::imshow("Image",image);

        // display result
//        cv::namedWindow("Segmented Image");
//        cv::imshow("Segmented Image",foreground);


//        waitKey();
//        return image;
        return foreground;
//        return std::make_tuple(foreground, binary);
    }



    Eigen::Matrix4f getTransformBetweenClouds(Ptr<cv::datasets::MSM_middlebury> &dataset, const int img1_num, const int img2_num) {

//        FILE_LOG(logINFO) << "R: " << stereo_util::infoMatrix(R) << R;
//        FILE_LOG(logINFO) << "T: " << stereo_util::infoMatrix(T) << T;

        Ptr<cv::datasets::MSM_middleburyObj> data_img1 =
                static_cast< Ptr<cv::datasets::MSM_middleburyObj> >  (dataset->getTrain()[img1_num]);
        Ptr<cv::datasets::MSM_middleburyObj> data_img2 =
                static_cast< Ptr<cv::datasets::MSM_middleburyObj> >  (dataset->getTrain()[img2_num]);


        Mat r1 = Mat(data_img1->r);
        Mat r2 = Mat(data_img2->r);

        // init translation vectors from dataset
        Mat t1 = Mat(3, 1, CV_64FC1, &data_img1->t);
        Mat t2 = Mat(3, 1, CV_64FC1, &data_img2->t);

        // rotation between img2 and img1
        Mat R = r2*r1.t();
        // translation between img2 and img1
        Mat T = t1 - (R.t()*t2 );

        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();

//        FILE_LOG(logINFO) << "R float: " <<  R.at<float>(0,0) << " double:" << R.at<double>(0,0);

        transformMatrix (0,0) = R.at<double>(0,0);
        transformMatrix (0,1) = R.at<double>(0,1);
        transformMatrix (0,2) = R.at<double>(0,2);

        transformMatrix (1,0) = R.at<double>(1,0);
        transformMatrix (1,1) = R.at<double>(1,1);
        transformMatrix (1,2) = R.at<double>(1,2);
        transformMatrix (2,0) = R.at<double>(2,0);
        transformMatrix (2,1) = R.at<double>(2,1);
        transformMatrix (2,2) = R.at<double>(2,2);

        transformMatrix (0,3) = T.at<double>(0);
        transformMatrix (1,3) = T.at<double>(1);
        transformMatrix (2,3) = T.at<double>(2);

        return transformMatrix;
    }

    Eigen::Matrix4f getTransformBetweenCloudsTsukuba(Ptr<cv::datasets::tsukuba_dataset> &dataset, const int frame_reference, const int current_frame) {

//        FILE_LOG(logINFO) << "R: " << stereo_util::infoMatrix(R) << R;
//        FILE_LOG(logINFO) << "T: " << stereo_util::infoMatrix(T) << T;

        Ptr<cv::datasets::tsukuba_datasetObj> data_img1 =
                static_cast< Ptr<cv::datasets::tsukuba_datasetObj> >  (dataset->getTrain()[frame_reference]);
        Ptr<cv::datasets::tsukuba_datasetObj> data_img2 =
                static_cast< Ptr<cv::datasets::tsukuba_datasetObj> >  (dataset->getTrain()[current_frame]);


        FILE_LOG(logINFO) << "Current Frame R and T: " << current_frame ;

        Mat r1 = Mat(data_img1->r);
        Mat r2 = Mat(data_img2->r);

        // init translation vectors from dataset
        Mat t1 = Mat(3, 1, CV_64FC1, &data_img1->tl);
        Mat t2 = Mat(3, 1, CV_64FC1, &data_img2->tl);

        // rotation between img2 and img1
        Mat R = r2.t(); //r2*r1.t();
        // translation between img2 and img1
        Mat T = -t2; //t1 - (R.t()*t2 );

        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();

//        FILE_LOG(logINFO) << "R float: " <<  R.at<float>(0,0) << " double:" << R.at<double>(0,0);

        transformMatrix (0,0) = R.at<double>(0,0);
        transformMatrix (0,1) = R.at<double>(0,1);
        transformMatrix (0,2) = R.at<double>(0,2);

        transformMatrix (1,0) = R.at<double>(1,0);
        transformMatrix (1,1) = R.at<double>(1,1);
        transformMatrix (1,2) = R.at<double>(1,2);
        transformMatrix (2,0) = R.at<double>(2,0);
        transformMatrix (2,1) = R.at<double>(2,1);
        transformMatrix (2,2) = R.at<double>(2,2);

        transformMatrix (0,3) = T.at<double>(0);
        transformMatrix (1,3) = T.at<double>(1);
        transformMatrix (2,3) = T.at<double>(2);

        return transformMatrix;
    }


    cv::Mat createPINVFromRT(cv::Mat R, cv::Mat T) {

        Mat P = Mat(4, 4, CV_64FC1);
        Mat h_concat;
        cv::hconcat(R, T, h_concat);
        Mat vect = Mat::zeros(1, 4, CV_64F);
        vect.at<double>(0, 3) = 1;
        cv::vconcat(h_concat, vect, P);
        FILE_LOG(logINFO) << "P matrix: " << P ;
        return P.inv();
    }
}
// Usage: ./app input.jpg
// g++ -ggdb `pkg-config --cflags opencv` -o `basename watershed.cpp .cpp` watershed.cpp `pkg-config --libs opencv`
#include "opencv2/opencv.hpp"
#include <string>

using namespace cv;
using namespace std;

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
            cv::MORPH_RECT, cv::Size(11, 11));
    cv::morphologyEx(binary, binary,
            cv::MORPH_CLOSE, structure_elem);

//        imshow("filtrata1",binary);

    vector< vector<Point> > contours; // Vector for storing contour
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
    imshow("filtrata",result1);
    cv::waitKey(0);

//        imshow("filtrata-c",binary_contour);


    return result1;
//

}

cv::Mat grabcut(cv::Mat image){

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
            200,        // number of iterations
            cv::GC_INIT_WITH_RECT); // use rectangle
    // Get the pixels marked as likely foreground
    cv::compare(result, cv::GC_PR_FGD, result, cv::CMP_EQ);
    // Generate output image
    cv::Mat foreground(image.size(),CV_8UC3,cv::Scalar(255,255,255));
    image.copyTo(foreground, result); // bg pixels not copied

    // draw rectangle on original image
    cv::rectangle(image, rectangle, cv::Scalar(255,255,255),1);
    cv::namedWindow("Image");
    cv::imshow("Image",image);

    // display result
    cv::namedWindow("Segmented Image");
    cv::imshow("Segmented Image",foreground);


    waitKey();
    return image;
}

int main(int argc, char* argv[])
{
    cv::Mat image = cv::imread(argv[1]);

//    cv::Mat segmented = segmentation(image);

    grabcut(image);

//    cv::imwrite("fatta.png", <#(_InputArray const &)img#>, <#(vector<int> const &)params#>)
//    cv::Mat binary;// = cv::imread(argv[2], 0);
//    cv::cvtColor(image, binary, CV_BGR2GRAY);
//    cv::threshold(binary, binary, 57, 255, THRESH_BINARY);
//
//    imshow("originalimage", image);
//    imshow("originalbinary", binary);
//
//    // Eliminate noise and smaller objects
//    cv::Mat fg;
//    cv::erode(binary,fg,cv::Mat(),cv::Point(-1,-1),2);
//    imshow("fg", fg);
//
//    // Identify image pixels without objects
//    cv::Mat bg;
//    cv::dilate(binary,bg,cv::Mat(),cv::Point(-1,-1),3);
//    cv::threshold(bg,bg,1, 128,cv::THRESH_BINARY_INV);
//    imshow("bg", bg);
//
//    // Create markers image
//    cv::Mat markers(binary.size(),CV_8U,cv::Scalar(0));
//    markers= fg+bg;
//    imshow("markers", markers);
//
//    // Create watershed segmentation object
//    WatershedSegmenter segmenter;
//    segmenter.setMarkers(markers);
//
//    cv::Mat result = segmenter.process(image);
//    result.convertTo(result,CV_8U);
//    imshow("final_result", result);
//
//    cv::waitKey(0);

    return 0;
}


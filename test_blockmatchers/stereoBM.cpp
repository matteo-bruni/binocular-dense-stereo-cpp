// g++ -ggdb `pkg-config --cflags opencv` -o `basename stereoBM.cpp .cpp` stereoBM.cpp `pkg-config --libs opencv`


#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "opencv2/photo/photo.hpp"

using namespace cv;
using namespace std;

int window_size = 9;
int temp1;
int number_of_disparities = 90;
int temp2;
int pre_filter_size = 11;
int temp3;
int pre_filter_cap = 63;
int temp4;
int min_disparity = 30;
int temp5;
int texture_threshold = 4;
int temp6;
int uniqueness_ratio = 1;
int temp7;
int max_diff = 8;
float temp8;
int speckle_window_size = 0;
int temp9;
int smooth_kernel = 7;
int temp_smooth_kernel;
int smooth_sigma_color = 66;
int temp_smooth_sigma_color;
int smooth_sigma_space = 17;
int temp_smooth_sigma_space;

int main(int argc, char *argv[]) {
    Mat img1, img2, g1, g2;
    Mat disp, disp8;
    Mat dispHeath;

    img1 = imread(argv[1]);
    img2 = imread(argv[2]);
    cvtColor(img1, g1, CV_BGR2GRAY);
    cvtColor(img2, g2, CV_BGR2GRAY);
    int i1;
    int i2;
    int i3;
    int i4;
    int i5;
    int i6;
    int i7;
    int i8;
    int i9;
    int i10;
    int i11;
    int i12;

    namedWindow("disp");
    createTrackbar("WindowSize", "disp", &window_size, 255, NULL);
    createTrackbar("no_of_disparities", "disp", &number_of_disparities, 255, NULL);
    createTrackbar("min_disparity (NEGATIVE!)", "disp", &min_disparity, 60, NULL);
    createTrackbar("filter_size", "disp", &pre_filter_size, 255, NULL);
    createTrackbar("filter_cap", "disp", &pre_filter_cap, 63, NULL);
    createTrackbar("texture_thresh", "disp", &texture_threshold, 2000, NULL);
    createTrackbar("uniquness", "disp", &uniqueness_ratio, 30, NULL);
    createTrackbar("disp12MaxDiff", "disp", &max_diff, 100, NULL);
    createTrackbar("Speckle_Window", "disp", &speckle_window_size, 5000, NULL);

    createTrackbar("Smooth_Kernel", "disp", &smooth_kernel, 100, NULL);
    createTrackbar("Smooth_colorsize", "disp", &smooth_sigma_color, 500, NULL);
    createTrackbar("Smooth_sigmasize", "disp", &smooth_sigma_space, 500, NULL);

    while (1) {
        i1 = window_size;
        StereoBM sbm;
        if (i1 % 2 == 0 && i1 >= 7) {
            temp1 = i1 - 1;
            sbm.state->SADWindowSize = temp1;
        }
        if (i1 < 7) {
            temp1 = 7;
            sbm.state->SADWindowSize = temp1;
        }
        if (i1 % 2 != 0 && i1 >= 7) {
            temp1 = i1;
            sbm.state->SADWindowSize = temp1;
        }


        i2 = number_of_disparities;
        if (i2 % 16 != 0 && i2 > 16) {
            temp2 = i2 - i2 % 16;
            sbm.state->numberOfDisparities = temp2;
        }
        if (i2 % 16 == 0 && i2 > 16) {
            temp2 = i2;
            sbm.state->numberOfDisparities = temp2;
        }
        if (i2 <= 16) {
            temp2 = 16;
            sbm.state->numberOfDisparities = temp2;

        }


        i3 = pre_filter_size;
        if (i3 % 2 == 0 && i3 >= 7) {
            temp3 = i3 - 1;
            sbm.state->preFilterSize = temp3;
        }
        if (i3 < 7) {
            temp3 = 7;
            sbm.state->preFilterSize = temp3;

        }
        if (i3 % 2 != 0 && i3 >= 7) {
            temp3 = i3;
            sbm.state->preFilterSize = temp3;
        }


        i4 = pre_filter_cap;
        if (i4 > 0) {
            temp4 = i4;
            sbm.state->preFilterCap = temp4;
        }
        if (i4 == 0) {
            temp4 = 1;
            sbm.state->preFilterCap = temp4;
        }


        i5 = min_disparity;
        temp5 = -i5;
        sbm.state->minDisparity = temp5;


        i6 = texture_threshold;
        temp6 = i6;
        sbm.state->textureThreshold = temp6;


        i7 = uniqueness_ratio;
        temp7 = i7;
        sbm.state->uniquenessRatio = temp7;


        i8 = max_diff;
        temp8 = i8;//0.01*((float)i8);
        sbm.state->disp12MaxDiff = temp8;


        i9 = speckle_window_size;
        temp9 = i9;
        sbm.state->speckleWindowSize = temp9;


        sbm.state->speckleRange = 8;

        sbm(g1, g2, disp, CV_32F);
        imshow("left", img1);
        imshow("right", img2);
        //imshow("disp", disp8);

        // APPLY MORPH OPENING


        Mat disp_smooth;


        i10 = smooth_kernel;
        if (i10 == 0)
            temp_smooth_kernel = 1;

        else {
            if (i10 % 2 == 0)
                temp_smooth_kernel = i10 - 1;
            else
                temp_smooth_kernel = i10;

        }


        i11 = smooth_sigma_color;
        temp_smooth_sigma_color = i11;
        i12 = smooth_sigma_space;
        temp_smooth_sigma_space = i12;

        // APPLY BILATERAL SMOOTHING
//		cv::bilateralFilter ( disp, disp_smooth, temp_smooth_kernel, temp_smooth_sigma_color, temp_smooth_sigma_space );			// size sigmacolor sigmaspace
//		normalize(disp_smooth, disp8, 0, 255, CV_MINMAX, CV_8U);
//		imshow("disp", disp8);

        // APPLY ADAPTIVE BILATERAL
        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
        cv::adaptiveBilateralFilter(disp8, disp_smooth, cv::Size(temp_smooth_kernel, temp_smooth_kernel), temp_smooth_sigma_space, temp_smooth_sigma_color); // size sigmaspace sigmacolor
        imshow("disp2", disp_smooth);



        //APPLY INPAINTING
        Mat inpaintMask;
        Mat img = Mat(disp_smooth.rows, disp_smooth.cols, CV_8U);


        img = disp_smooth.clone();
        inpaintMask = Mat::zeros(img.size(), CV_8U);
        imshow("inpaintMask", img);

        for (int rows = 0; rows < img.rows; ++rows) {
            for (int cols = 0; cols < img.cols; ++cols) {
                if ((img.at<unsigned char>(rows, cols)) > 150)
                    inpaintMask.at<unsigned char>(rows, cols) = 255;
            }

        }
        imshow("inpaintMask", inpaintMask);

        Mat inpainted;
        cv::inpaint(img, inpaintMask, inpainted, 3, INPAINT_TELEA);

        imshow("inpainted image", inpainted);


        cv::adaptiveBilateralFilter(inpainted, disp_smooth, cv::Size(temp_smooth_kernel, temp_smooth_kernel), temp_smooth_sigma_space, temp_smooth_sigma_color); // size sigmaspace sigmacolor



        applyColorMap(disp_smooth, dispHeath, COLORMAP_JET);

        imshow("inpainted image- smooth", dispHeath);

//		applyColorMap(disp8, dispHeath, COLORMAP_JET);



        waitKey(1);
    }

    return (0);
}

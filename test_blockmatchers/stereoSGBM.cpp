// g++ -ggdb `pkg-config --cflags opencv` -o `basename stereoSGBM.cpp .cpp` stereoSGBM.cpp `pkg-config --libs opencv`


#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>
using namespace cv;
using namespace std;

int window_size = 9;
int temp1;
int number_of_disparities = 80;
int temp2;
int pre_filter_size = 5;
int temp3;
int pre_filter_cap = 23;
int temp4;
int min_disparity = 30;
int temp5;
int texture_threshold = 500;
int temp6;
int uniqueness_ratio = 0;
int temp7;
int max_diff = -1;
float temp8;
int speckle_window_size = 0;
int temp9;
int speckle_range = 0;
int temp10;

int number_of_image_channels = 3;

            
int main(int argc, char* argv[])
{
    Mat img1, img2, g1, g2;
    Mat dispHeath;
    Mat disp, disp8;
    img1 = imread(argv[1]);
    img2 = imread(argv[2]);
    img1.copyTo(g1);
    img2.copyTo(g2);
    //cvtColor(img1, g1, CV_BGR2GRAY);
    //cvtColor(img2, g2, CV_BGR2GRAY);
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
	namedWindow("disp");
	createTrackbar("WindowSize", "disp", &window_size,255, NULL);
	createTrackbar("no_of_disparities", "disp", &number_of_disparities,255, NULL);
	createTrackbar("min_disparity", "disp", &min_disparity,60, NULL);

	//createTrackbar("filter_size", "disp", &pre_filter_size,255, NULL);
	createTrackbar("filter_cap", "disp", &pre_filter_cap,63, NULL);
	//createTrackbar("texture_thresh", "disp", &texture_threshold,2000, NULL);
	createTrackbar("uniquness", "disp", &uniqueness_ratio,30, NULL);
	createTrackbar("disp12MaxDiff", "disp", &max_diff,100, NULL);
	createTrackbar("Speckle Window size (50-200)", "disp", &speckle_window_size, 200, NULL);
	createTrackbar("Speckle Window range (1-2)", "disp", &speckle_range,10, NULL);

	
   while(1)
	{ 
		i1 = window_size;
        StereoSGBM sbm;
		if(i1%2==0 && i1>=7)
		{
			temp1 = i1-1;		
			sbm.SADWindowSize = temp1;
		}	
		if(i1<7)
		{
			temp1 =	7;	
			sbm.SADWindowSize = temp1;
		}	
		if(i1%2!=0 && i1>=7)
	        {
			temp1 =	i1;
			sbm.SADWindowSize = temp1;
		}

	
		i2 = number_of_disparities;
		if(i2%16!=0 && i2>16)
		{
			temp2 = i2 - i2%16;		
			sbm.numberOfDisparities = temp2;
		}
		if(i2%16==0 && i2>16)
	    {
			temp2 =	i2;
			sbm.numberOfDisparities = temp2;
		}
		if(i2<=16)
		{
			temp2 =	16;	
			sbm.numberOfDisparities = temp2;
			
		}

		
		
		
		i4 = pre_filter_cap;
		if(i4>0)
		{
			temp4 = i4;
			sbm.preFilterCap = temp4;
		}
		if(i4==0)
		{
			temp4 = 1;
			sbm.preFilterCap = temp4;
		}
	        
	       
		i5 = min_disparity;
		temp5 = -i5;
		sbm.minDisparity = temp5;



		i7 = uniqueness_ratio;
		temp7 = i7;
		sbm.uniquenessRatio = temp7;
		
	    
		i8 = max_diff;
		temp8 = i8;
		sbm.disp12MaxDiff = temp8;   


		i9 = speckle_window_size;
		temp9 = i9;
		sbm.speckleWindowSize = temp9;


		i10 = speckle_range;
		temp10 = i10;
		sbm.speckleRange = temp10;

        sbm.fullDP = false;
		sbm.P1 = 8*number_of_image_channels*temp1*temp1;
        sbm.P2 = 8*32*temp1*temp1;
	    
	    sbm(g1, g2, disp);
	    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
	    imshow("left", img1);
	    imshow("right", img2);

	    applyColorMap(disp8, dispHeath, COLORMAP_JET);

	    imshow("disp", dispHeath);

		Mat inpaintMask;
		Mat img = Mat(disp8.rows, disp8.cols, CV_8U);


		img = disp8.clone();
		inpaintMask = Mat::zeros(img.size(), CV_8U);
		imshow("inpaintMask", img);

		for (int rows = 0; rows < img.rows; ++rows) {
			for (int cols = 0; cols < img.cols; ++cols) {
				if ((img.at<unsigned char>(rows, cols)) > 180)
					inpaintMask.at<unsigned char>(rows, cols) = 255;
			}

		}
		imshow("inpaintMask", inpaintMask);

		Mat inpainted;
		cv::inpaint(img, inpaintMask, inpainted, 5, INPAINT_TELEA);

		imshow("inpainted image", inpainted);
		waitKey(5);
	}
    return(0);
}
